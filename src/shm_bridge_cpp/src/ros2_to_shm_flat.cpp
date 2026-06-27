// ros2_to_shm_flat.cpp — one topic -> SHM, one process per stream, with TYPED
// zero-copy FLAT extraction for the 25 heavy types (flat_adapter.hpp) and a
// universal CDR fallback (GenericSubscription) for everything else.
//
// Same low-latency contract as ros2_to_shm; this one adds the C++ FLAT fast path
// so image/cloud/tensor payloads are written with a single zero-copy memcpy and
// reconstructed as ndarrays by the reader (matching the Python adapter).
//
// Usage:  ros2 run shm_bridge_cpp ros2_to_shm_flat --ros-args -p topic:=/camera/rgb
#include "shm_bridge_cpp/shm_contract.hpp"
#include "shm_bridge_cpp/flat_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <thread>

using namespace shm_contract;
namespace fa = flat_adapter;

static std::string topic_to_name(const std::string& topic) {
    std::string s = topic;
    while (!s.empty() && s.front() == '/') s.erase(s.begin());
    std::string out;
    for (char c : s) { if (c == '/') out += "__"; else out += c; }
    return out.empty() ? "root" : out;
}

class Ros2ToShmFlat : public rclcpp::Node {
public:
    Ros2ToShmFlat() : Node("ros2_to_shm_flat_cpp") {
        topic_ = declare_parameter<std::string>("topic", "/rgb");
        std::string nm = declare_parameter<std::string>("name", "");
        std::string forced = declare_parameter<std::string>("type", "");
        max_bytes_ = static_cast<size_t>(declare_parameter<int>("max_bytes", 1920 * 1080 * 4));
        double timeout = declare_parameter<double>("resolve_timeout", 30.0);

        name_ = nm.empty() ? topic_to_name(topic_) : nm;
        type_name_ = forced.empty() ? resolve_type(topic_, timeout) : forced;
        if (type_name_.empty()) throw std::runtime_error("type resolution failed");

        size_t hs = 0, fs = 0;
        hdr_ = map_file(header_path(name_), HEADER_SIZE, true, hs);
        frame_ = map_file(frame_path(name_), max_bytes_, true, fs);
        frame_cap_ = fs;
        Header h{}; set_type_name(h, type_name_);
        std::memcpy(hdr_, &h, HEADER_SIZE);

        if (!subscribe_flat(type_name_)) {
            // CDR fallback for any non-heavy type
            gsub_ = create_generic_subscription(
                topic_, type_name_, rclcpp::QoS(1),
                [this](std::shared_ptr<rclcpp::SerializedMessage> m) { on_cdr(m); });
            RCLCPP_INFO(get_logger(), "[%s] %s (%s) CDR -> /dev/shm/%s_*",
                        name_.c_str(), topic_.c_str(), type_name_.c_str(), name_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "[%s] %s (%s) FLAT -> /dev/shm/%s_*",
                        name_.c_str(), topic_.c_str(), type_name_.c_str(), name_.c_str());
        }
    }

private:
    // Create a typed subscription for type T, extracting FLAT via `ex`.
    template <typename T>
    void make_flat_sub(std::function<fa::Flat(const T&)> ex) {
        flat_sub_holder_ = create_subscription<T>(
            topic_, rclcpp::QoS(1),
            [this, ex](std::shared_ptr<const T> msg) {
                fa::Flat f = ex(*msg);
                if (f.ok) write_flat(f);
            });
    }

    // Register all 25 heavy types. Returns true if matched (typed FLAT wired).
    bool subscribe_flat(const std::string& t) {
        using namespace sensor_msgs::msg;
        using namespace std_msgs::msg;
        if (t == "sensor_msgs/msg/Image") { make_flat_sub<Image>(fa::image); return true; }
        if (t == "sensor_msgs/msg/CompressedImage") { make_flat_sub<CompressedImage>(fa::compressed_image); return true; }
        if (t == "sensor_msgs/msg/PointCloud2") { make_flat_sub<PointCloud2>(fa::pointcloud2); return true; }
        if (t == "sensor_msgs/msg/LaserScan") { make_flat_sub<LaserScan>(fa::laserscan); return true; }
        if (t == "stereo_msgs/msg/DisparityImage") { make_flat_sub<stereo_msgs::msg::DisparityImage>(fa::disparity); return true; }
        if (t == "nav_msgs/msg/OccupancyGrid") { make_flat_sub<nav_msgs::msg::OccupancyGrid>(fa::occupancy_grid); return true; }
        if (t == "map_msgs/msg/OccupancyGridUpdate") { make_flat_sub<map_msgs::msg::OccupancyGridUpdate>(fa::occupancy_grid_update); return true; }
        if (t == "octomap_msgs/msg/Octomap") { make_flat_sub<octomap_msgs::msg::Octomap>(fa::octomap); return true; }
        if (t == "visualization_msgs/msg/MeshFile") { make_flat_sub<visualization_msgs::msg::MeshFile>(fa::mesh_file); return true; }
        if (t == "rmw_dds_common/msg/Gid") { make_flat_sub<rmw_dds_common::msg::Gid>(fa::gid); return true; }
        if (t == "std_msgs/msg/ByteMultiArray")    { make_flat_sub<ByteMultiArray>([](const ByteMultiArray& m){ return fa::multiarray<ByteMultiArray, uint8_t>(m, fa::D_U8); }); return true; }
        if (t == "std_msgs/msg/UInt8MultiArray")   { make_flat_sub<UInt8MultiArray>([](const UInt8MultiArray& m){ return fa::multiarray<UInt8MultiArray, uint8_t>(m, fa::D_U8); }); return true; }
        if (t == "std_msgs/msg/Int8MultiArray")    { make_flat_sub<Int8MultiArray>([](const Int8MultiArray& m){ return fa::multiarray<Int8MultiArray, int8_t>(m, fa::D_I8); }); return true; }
        if (t == "std_msgs/msg/UInt16MultiArray")  { make_flat_sub<UInt16MultiArray>([](const UInt16MultiArray& m){ return fa::multiarray<UInt16MultiArray, uint16_t>(m, fa::D_U16); }); return true; }
        if (t == "std_msgs/msg/Int16MultiArray")   { make_flat_sub<Int16MultiArray>([](const Int16MultiArray& m){ return fa::multiarray<Int16MultiArray, int16_t>(m, fa::D_I16); }); return true; }
        if (t == "std_msgs/msg/UInt32MultiArray")  { make_flat_sub<UInt32MultiArray>([](const UInt32MultiArray& m){ return fa::multiarray<UInt32MultiArray, uint32_t>(m, fa::D_U32); }); return true; }
        if (t == "std_msgs/msg/Int32MultiArray")   { make_flat_sub<Int32MultiArray>([](const Int32MultiArray& m){ return fa::multiarray<Int32MultiArray, int32_t>(m, fa::D_I32); }); return true; }
        if (t == "std_msgs/msg/Float32MultiArray") { make_flat_sub<Float32MultiArray>([](const Float32MultiArray& m){ return fa::multiarray<Float32MultiArray, float>(m, fa::D_F32); }); return true; }
        if (t == "std_msgs/msg/Float64MultiArray") { make_flat_sub<Float64MultiArray>([](const Float64MultiArray& m){ return fa::multiarray<Float64MultiArray, double>(m, fa::D_F64); }); return true; }
        // UInt64/Int64 MultiArray: dtype ids exist but reader maps to u32/i32; skip
        // typed FLAT and let them go CDR for correctness.
        return false;
    }

    std::string resolve_type(const std::string& topic, double timeout) {
        auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(timeout);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            auto names = get_topic_names_and_types();
            auto it = names.find(topic);
            if (it != names.end() && !it->second.empty()) {
                RCLCPP_INFO(get_logger(), "resolved %s -> %s",
                            topic.c_str(), it->second[0].c_str());
                return it->second[0];
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "waiting for %s...", topic.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        return "";
    }

    uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void write_flat(const fa::Flat& f) {
        if (f.len > frame_cap_) { warn_cap(f.len); return; }
        uint64_t ts = now_ns();
        if (!recipe_written_) {
            write_recipe_flat(f, ts);
            recipe_written_ = true;
        }
        seq_ += 1; store_seq_release(hdr_, seq_);
        std::memcpy(frame_, f.ptr, f.len);
        Header h{};
        h.seq = seq_; h.encoding_id = ENC_FLAT; h.data_size = static_cast<uint32_t>(f.len);
        h.width = f.width; h.height = f.height; h.channels = f.channels;
        h.dtype_id = f.dtype_id; h.timestamp_ns = ts;
        set_type_name(h, type_name_);
        std::memcpy(hdr_, &h, HEADER_SIZE);
        seq_ += 1; store_seq_release(hdr_, seq_);
    }

    void on_cdr(std::shared_ptr<rclcpp::SerializedMessage> m) {
        const auto& rcl = m->get_rcl_serialized_message();
        size_t n = rcl.buffer_length;
        if (n > frame_cap_) { warn_cap(n); return; }
        uint64_t ts = now_ns();
        if (!recipe_written_) { write_recipe_cdr(ts); recipe_written_ = true; }
        seq_ += 1; store_seq_release(hdr_, seq_);
        std::memcpy(frame_, rcl.buffer, n);
        Header h{};
        h.seq = seq_; h.encoding_id = ENC_CDR; h.data_size = static_cast<uint32_t>(n);
        h.timestamp_ns = ts; set_type_name(h, type_name_);
        std::memcpy(hdr_, &h, HEADER_SIZE);
        seq_ += 1; store_seq_release(hdr_, seq_);
    }

    void warn_cap(size_t n) {
        RCLCPP_WARN(get_logger(), "[%s] %zu > cap %zu, dropped",
                    name_.c_str(), n, frame_cap_);
    }

    void write_recipe_flat(const fa::Flat& f, uint64_t ts) {
        static const char* DT[] = {"uint8","int8","uint16","int16","uint32","int32","float32","float64"};
        std::ostringstream o;
        o << "{\"topic\":\"" << topic_ << "\",\"type_name\":\"" << type_name_
          << "\",\"encoding\":\"flat\",\"dtype\":\"" << DT[f.dtype_id] << "\",\"shape\":[";
        if (f.channels > 1) o << f.height << "," << f.width << "," << f.channels;
        else if (f.height > 1) o << f.height << "," << f.width;
        else o << f.width;
        o << "],\"timestamp_ns\":" << ts << "}";
        save(o.str());
    }
    void write_recipe_cdr(uint64_t ts) {
        std::ostringstream o;
        o << "{\"topic\":\"" << topic_ << "\",\"type_name\":\"" << type_name_
          << "\",\"encoding\":\"cdr\",\"timestamp_ns\":" << ts << "}";
        save(o.str());
    }
    void save(const std::string& s) {
        std::string tmp = recipe_path(name_) + ".tmp";
        { std::ofstream f(tmp); f << s; }
        std::rename(tmp.c_str(), recipe_path(name_).c_str());
    }

    std::string topic_, name_, type_name_;
    size_t max_bytes_ = 0, frame_cap_ = 0;
    void* hdr_ = nullptr; void* frame_ = nullptr;
    uint32_t seq_ = 0;
    bool recipe_written_ = false;
    rclcpp::SubscriptionBase::SharedPtr flat_sub_holder_;
    rclcpp::GenericSubscription::SharedPtr gsub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Ros2ToShmFlat>());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[ros2_to_shm_flat] %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

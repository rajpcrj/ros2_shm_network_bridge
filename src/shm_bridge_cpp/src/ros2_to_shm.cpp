// ros2_to_shm.cpp — ONE topic -> shared memory, one OS process per stream (C++).
//
// Mirror of the Python ros2_to_shm with the same low-latency contract:
//   * single-threaded, one process per stream (launch N for N streams)
//   * type auto-resolved from the ROS graph (no need to pass it)
//   * GenericSubscription -> CDR for ANY type (universal); payload memcpy'd once
//   * recipe JSON written only on structural change; data_size rides the header
//
// FLAT (zero-copy) extraction for image-like types is provided by the Python
// writer and the dedicated ros_rgb_depth_to_shm node; this C++ node is the
// universal CDR transport with auto-discovery, sharing the identical byte format.
//
// Usage:
//   ros2 run shm_bridge_cpp ros2_to_shm --ros-args -p topic:=/rgb
//   ros2 run shm_bridge_cpp ros2_to_shm --ros-args -p topic:=/rgb -p name:=rgb
#include "shm_bridge_cpp/shm_contract.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

using namespace shm_contract;

static std::string topic_to_name(const std::string& topic) {
    std::string s = topic;
    while (!s.empty() && s.front() == '/') s.erase(s.begin());
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '/') { out += "__"; }
        else { out += s[i]; }
    }
    return out.empty() ? "root" : out;
}

class Ros2ToShm : public rclcpp::Node {
public:
    Ros2ToShm() : Node("ros2_to_shm_cpp") {
        topic_ = declare_parameter<std::string>("topic", "/rgb");
        std::string name = declare_parameter<std::string>("name", "");
        std::string forced = declare_parameter<std::string>("type", "");
        max_bytes_ = static_cast<size_t>(declare_parameter<int>("max_bytes", 1920 * 1080 * 4));
        double timeout = declare_parameter<double>("resolve_timeout", 30.0);

        name_ = name.empty() ? topic_to_name(topic_) : name;
        type_name_ = forced.empty() ? resolve_type(topic_, timeout) : forced;
        if (type_name_.empty()) {
            RCLCPP_ERROR(get_logger(), "could not resolve type for %s", topic_.c_str());
            throw std::runtime_error("type resolution failed");
        }

        size_t hs = 0, fs = 0;
        hdr_ = map_file(header_path(name_), HEADER_SIZE, true, hs);
        frame_ = map_file(frame_path(name_), max_bytes_, true, fs);
        frame_cap_ = fs;
        Header h{};
        set_type_name(h, type_name_);
        std::memcpy(hdr_, &h, HEADER_SIZE);

        sub_ = create_generic_subscription(
            topic_, type_name_, rclcpp::QoS(1),
            [this](std::shared_ptr<rclcpp::SerializedMessage> m) { on_msg(m); });

        RCLCPP_INFO(get_logger(), "[%s] %s (%s) -> /dev/shm/%s_*",
                    name_.c_str(), topic_.c_str(), type_name_.c_str(), name_.c_str());
    }

private:
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
                                 "waiting for %s to appear...", topic.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        return "";
    }

    void on_msg(std::shared_ptr<rclcpp::SerializedMessage> m) {
        const auto& rcl = m->get_rcl_serialized_message();
        size_t n = rcl.buffer_length;
        if (n > frame_cap_) {
            RCLCPP_WARN(get_logger(), "[%s] %zu > cap %zu, dropped",
                        name_.c_str(), n, frame_cap_);
            return;
        }
        uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (!recipe_written_) {                 // structural: write once (CDR fixed)
            write_recipe(recipe_path(name_), topic_, type_name_, ts);
            recipe_written_ = true;
        }

        // ---- SEQLOCK hot path ----
        seq_ += 1;
        store_seq_release(hdr_, seq_);
        std::memcpy(frame_, rcl.buffer, n);
        Header h{};
        h.seq = seq_;
        h.encoding_id = ENC_CDR;
        h.data_size = static_cast<uint32_t>(n);
        h.timestamp_ns = ts;
        set_type_name(h, type_name_);
        std::memcpy(hdr_, &h, HEADER_SIZE);
        seq_ += 1;
        store_seq_release(hdr_, seq_);
    }

    static void write_recipe(const std::string& path, const std::string& topic,
                             const std::string& type_name, uint64_t ts) {
        std::ostringstream o;
        o << "{\"topic\":\"" << topic << "\",\"type_name\":\"" << type_name
          << "\",\"encoding\":\"cdr\",\"timestamp_ns\":" << ts << "}";
        std::string tmp = path + ".tmp";
        { std::ofstream f(tmp); f << o.str(); }
        std::rename(tmp.c_str(), path.c_str());
    }

    std::string topic_, name_, type_name_;
    size_t max_bytes_ = 0, frame_cap_ = 0;
    void* hdr_ = nullptr;
    void* frame_ = nullptr;
    uint32_t seq_ = 0;
    bool recipe_written_ = false;
    rclcpp::GenericSubscription::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Ros2ToShm>());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[ros2_to_shm] %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

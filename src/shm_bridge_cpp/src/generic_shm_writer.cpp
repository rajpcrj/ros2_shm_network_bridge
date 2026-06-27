// generic_shm_writer.cpp — type-agnostic ROS 2 -> shared memory writer (PROMPT.md §5).
//
// Uses rclcpp::GenericSubscription so it can subscribe to ANY message type by
// name at runtime (no compile-time type dependency). Every message is written via
// the CDR path (universal). The Python writer covers the FLAT zero-copy fast path
// for image-like types; this C++ node is the universal-transport counterpart and
// shares the exact same binary header + recipe contract.
//
// Parameters:
//   streams (string[])  "stream_name:topic:Type" e.g.
//        ["tf:/tf:tf2_msgs/msg/TFMessage", "rgb:/rgb:sensor_msgs/msg/Image"]
//   max_bytes (int)     per-stream frame cap (default 1920*1080*4)
#include "shm_bridge_cpp/shm_contract.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace shm_contract;

struct Stream {
    std::string name, topic, type_name, recipe_path;
    void* hdr = nullptr;
    void* frame = nullptr;
    size_t frame_cap = 0;
    uint32_t seq = 0;
    rclcpp::GenericSubscription::SharedPtr sub;
};

static std::vector<std::string> split3(const std::string& s) {
    // split on first two ':' -> name, topic, type
    auto a = s.find(':');
    auto b = s.find(':', a + 1);
    return {s.substr(0, a), s.substr(a + 1, b - a - 1), s.substr(b + 1)};
}

class GenericSHMWriter : public rclcpp::Node {
public:
    GenericSHMWriter() : Node("generic_shm_writer_cpp") {
        declare_parameter<std::vector<std::string>>(
            "streams", {"tf:/tf:tf2_msgs/msg/TFMessage"});
        declare_parameter<int>("max_bytes", 1920 * 1080 * 4);

        max_bytes_ = static_cast<size_t>(get_parameter("max_bytes").as_int());
        auto specs = get_parameter("streams").as_string_array();

        for (auto& spec : specs) {
            auto parts = split3(spec);
            auto st = std::make_shared<Stream>();
            st->name = parts[0];
            st->topic = parts[1];
            st->type_name = parts[2];
            st->recipe_path = recipe_path(st->name);

            size_t hs = 0, fs = 0;
            st->hdr = map_file(header_path(st->name), HEADER_SIZE, true, hs);
            st->frame = map_file(frame_path(st->name), max_bytes_, true, fs);
            st->frame_cap = fs;

            Header h{};
            set_type_name(h, st->type_name);
            std::memcpy(st->hdr, &h, HEADER_SIZE);  // clean even seq=0

            st->sub = create_generic_subscription(
                st->topic, st->type_name, rclcpp::QoS(1),
                [this, st](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    on_msg(st, msg);
                });

            RCLCPP_INFO(get_logger(), "[%s] %s (%s) -> /dev/shm/%s_*",
                        st->name.c_str(), st->topic.c_str(),
                        st->type_name.c_str(), st->name.c_str());
            streams_.push_back(st);
        }
    }

private:
    void on_msg(std::shared_ptr<Stream> st,
                std::shared_ptr<rclcpp::SerializedMessage> msg) {
        const auto& rcl = msg->get_rcl_serialized_message();
        size_t n = rcl.buffer_length;
        if (n > st->frame_cap) {
            RCLCPP_WARN(get_logger(), "[%s] payload %zu > cap %zu, dropped",
                        st->name.c_str(), n, st->frame_cap);
            return;
        }
        uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // ---- SEQLOCK write (writer never blocks; PROMPT.md §5) ----
        st->seq += 1;                               // odd: writing
        store_seq_release(st->hdr, st->seq);

        std::memcpy(st->frame, rcl.buffer, n);

        Header h{};
        h.seq = st->seq;
        h.encoding_id = ENC_CDR;
        h.data_size = static_cast<uint32_t>(n);
        h.timestamp_ns = ts;
        set_type_name(h, st->type_name);
        std::memcpy(st->hdr, &h, HEADER_SIZE);

        write_recipe(st->recipe_path, st->type_name, n, ts);

        st->seq += 1;                               // even: stable
        store_seq_release(st->hdr, st->seq);
    }

    static void write_recipe(const std::string& path, const std::string& type_name,
                             size_t n, uint64_t ts) {
        std::ostringstream o;
        o << "{\"type_name\":\"" << type_name << "\",\"encoding\":\"cdr\","
          << "\"data_size\":" << n << ",\"timestamp_ns\":" << ts << "}";
        std::string tmp = path + ".tmp";
        { std::ofstream f(tmp); f << o.str(); }
        std::rename(tmp.c_str(), path.c_str());     // atomic swap
    }

    size_t max_bytes_ = 0;
    std::vector<std::shared_ptr<Stream>> streams_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericSHMWriter>());
    rclcpp::shutdown();
    return 0;
}

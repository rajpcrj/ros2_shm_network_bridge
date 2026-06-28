// shm_to_ros2.cpp — bridge /dev/shm -> ROS 2 (the READ side).
//
// Attaches to a shared-memory stream with shm_bridge::Reader, blocks ~0% CPU on
// the futex until a new frame arrives, reconstructs a sensor_msgs/Image and
// publishes it on a ROS 2 topic. This lets a non-ROS producer (or a faster SHM
// path) feed a normal ROS 2 graph.
//
// Pairs with ros2_to_shm.cpp: run that to fill /dev/shm/rgb_*, then this to
// expose it as /from_shm.
//
// Run:
//   ros2 run shm_bridge_cpp ex_shm_to_ros2 --ros-args -p stream:=rgb -p topic:=/from_shm
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

class ShmToRos2 : public rclcpp::Node {
public:
    ShmToRos2() : Node("shm_to_ros2") {
        stream_   = declare_parameter<std::string>("stream", "rgb");
        topic_    = declare_parameter<std::string>("topic", "/from_shm");
        encoding_ = declare_parameter<std::string>("encoding", "rgb8");

        pub_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());
        RCLCPP_INFO(get_logger(), "reading /dev/shm/%s_* -> publishing '%s'",
                    stream_.c_str(), topic_.c_str());

        // The Reader throws if the stream doesn't exist yet; retry until the
        // producer has created it.
        worker_ = std::thread([this] { run(); });
    }
    ~ShmToRos2() override { running_ = false; if (worker_.joinable()) worker_.join(); }

private:
    void run() {
        std::unique_ptr<shm_bridge::Reader> reader;
        while (running_ && rclcpp::ok() && !reader) {
            try { reader = std::make_unique<shm_bridge::Reader>(stream_); }
            catch (const std::exception&) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "waiting for /dev/shm/%s_* ...", stream_.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
        shm_bridge::Frame f;
        while (running_ && rclcpp::ok()) {
            // Block up to 100 ms (so we can notice shutdown), 0% CPU while idle.
            if (!reader->wait_and_read(f, 100ull * 1000 * 1000)) continue;

            sensor_msgs::msg::Image img;
            img.header.stamp = now();
            img.header.frame_id = f.type_name.empty() ? stream_ : f.type_name;
            img.height   = f.height ? f.height : 1;
            img.width    = f.width;
            img.encoding = encoding_;
            img.is_bigendian = 0;
            img.step     = f.width * std::max<uint32_t>(f.channels, 1);
            img.data.assign(f.data.begin(), f.data.end());   // single copy
            pub_->publish(std::move(img));
        }
    }

    std::string stream_, topic_, encoding_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::thread worker_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShmToRos2>());
    rclcpp::shutdown();
    return 0;
}

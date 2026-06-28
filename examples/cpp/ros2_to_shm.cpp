// ros2_to_shm.cpp — bridge ROS 2 -> /dev/shm (the WRITE side).
//
// Subscribes to a sensor_msgs/Image topic and republishes every frame into a
// shared-memory stream via shm_bridge::Writer. Any process on the same machine
// (C++ or Python, ROS or not) can then read it with ~0% CPU and sub-millisecond
// latency using shm_bridge::Reader / the Python contract.
//
// This is the canonical "producer". Swap the message type / topic for your own.
//
// Build (standalone, after you've installed the lib to /usr/local — see
// install_lib.sh and docs/03_build_and_install.md):
//   g++ -std=c++17 ros2_to_shm.cpp -o ros2_to_shm \
//       $(pkg-config --cflags --libs ...) -lshm_bridge_cpp        # see docs
// Or build it inside the ament package (it is wired into CMakeLists as an example).
//
// Run:
//   ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args -p topic:=/camera/image_raw -p stream:=rgb
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

class Ros2ToShm : public rclcpp::Node {
public:
    Ros2ToShm() : Node("ros2_to_shm") {
        topic_  = declare_parameter<std::string>("topic", "/camera/image_raw");
        stream_ = declare_parameter<std::string>("stream", "rgb");
        // Size the buffer for the largest frame you expect. 4K RGBA is a safe cap.
        const size_t max_bytes = declare_parameter<int>("max_bytes", 3840 * 2160 * 4);

        writer_ = std::make_unique<shm_bridge::Writer>(stream_, max_bytes);
        RCLCPP_INFO(get_logger(), "writing '%s' -> /dev/shm/%s_*  (cap=%zu B)",
                    topic_.c_str(), stream_.c_str(), max_bytes);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(m); });
    }

private:
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& m) {
        // FLAT publish: copy the raw pixel bytes once into shared memory along with
        // the shape so readers can reconstruct the image without deserializing.
        const bool ok = writer_->write_flat(
            m->data.data(), m->data.size(),
            m->width, m->height, /*channels*/ m->step ? m->step / std::max<uint32_t>(m->width, 1) : 1,
            shm_bridge::DType::U8, "sensor_msgs/msg/Image", topic_);
        if (!ok)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "write_flat failed (frame larger than buffer?)");
    }

    std::string topic_, stream_;
    std::unique_ptr<shm_bridge::Writer> writer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2ToShm>());
    rclcpp::shutdown();
    return 0;
}

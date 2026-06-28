// write_cpp.cpp — the absolute minimum to WRITE to shared memory in C++.
// No ROS. Just the library. Compile after installing the lib (see README):
//   g++ -std=c++17 write_cpp.cpp -o write_cpp -lshm_bridge_cpp -lpthread
// Run alongside read_cpp (or the Python reader) to see frames flow.
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

int main() {
    // One stream named "demo"; buffer big enough for a 640x480 RGB frame.
    shm_bridge::Writer w("demo", 640 * 480 * 3);

    std::vector<uint8_t> frame(640 * 480 * 3);
    uint8_t tick = 0;
    while (true) {
        // fill with a moving pattern so the reader sees it change
        std::memset(frame.data(), tick++, frame.size());

        // write_flat copies the bytes once into shared memory + records the shape.
        // Args: ptr, len, width, height, channels, dtype, ros-type-name, topic.
        w.write_flat(frame.data(), frame.size(),
                     640, 480, 3, shm_bridge::DType::U8,
                     "sensor_msgs/msg/Image", "/demo");

        std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 Hz
    }
}

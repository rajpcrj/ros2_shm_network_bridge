// example_usage.cpp — minimal demo of the shm_bridge C++ library.
//
// Shows the whole API with NO ROS: one Writer publishes a fake 4x4 RGB frame in a
// loop, one Reader attaches and reconstructs it. This is exactly what you'd embed
// in your own node — swap the fake buffer for real message data.
//
// Build: it's compiled as `example_usage` by this package's CMakeLists.
// Run two terminals:
//   ros2 run shm_bridge_cpp example_usage write
//   ros2 run shm_bridge_cpp example_usage read
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

static int do_write() {
    // 4x4 RGB image = 48 bytes; size the buffer generously.
    shm_bridge::Writer w("example", 1920 * 1080 * 4);
    std::printf("[write] publishing 4x4 RGB to /dev/shm/example_* (Ctrl-C to stop)\n");

    std::vector<uint8_t> img(4 * 4 * 3);
    uint8_t tick = 0;
    while (true) {
        for (size_t i = 0; i < img.size(); ++i) img[i] = static_cast<uint8_t>(i + tick);
        w.write_flat(img.data(), img.size(),
                     /*width*/4, /*height*/4, /*channels*/3,
                     shm_bridge::DType::U8, "sensor_msgs/msg/Image", "/example");
        ++tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

static int do_read() {
    shm_bridge::Reader r("example");      // throws if writer hasn't created it yet
    std::printf("[read] attached to /dev/shm/example_*\n");

    shm_bridge::Frame f;
    while (true) {
        if (r.read(f)) {
            std::printf("[read] seq=%u %s %ux%ux%u %zuB type=%s first3=[%u,%u,%u]\n",
                        f.seq,
                        f.encoding == shm_bridge::Encoding::FLAT ? "FLAT" : "CDR",
                        f.width, f.height, f.channels, f.data.size(),
                        f.type_name.c_str(),
                        f.data.size() > 2 ? f.data[0] : 0,
                        f.data.size() > 2 ? f.data[1] : 0,
                        f.data.size() > 2 ? f.data[2] : 0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "write") return do_write();
    if (mode == "read") return do_read();
    std::fprintf(stderr, "usage: %s write|read\n", argv[0]);
    return 1;
}

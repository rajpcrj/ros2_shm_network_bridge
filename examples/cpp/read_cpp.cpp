// read_cpp.cpp — the absolute minimum to READ from shared memory in C++.
// No ROS. Just the library. Compile after installing the lib (see README):
//   g++ -std=c++17 read_cpp.cpp -o read_cpp -lshm_bridge_cpp -lpthread
// Run it AFTER a writer (write_cpp, ros2_to_shm, or the Python writer) is up.
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <cstdio>

int main() {
    // Attaches to /dev/shm/demo_* . Throws if the writer hasn't created it yet.
    shm_bridge::Reader r("demo");
    shm_bridge::Frame f;

    while (true) {
        // wait_and_read blocks at ~0% CPU on a futex until a NEW frame arrives.
        // timeout_ns = 0 waits forever. Use r.read(f) instead for a non-blocking
        // snapshot of the latest frame.
        if (r.wait_and_read(f, 0)) {
            std::printf("seq=%u  %ux%ux%u  %zu bytes  type=%s  first byte=%u\n",
                        f.seq, f.width, f.height, f.channels,
                        f.data.size(), f.type_name.c_str(),
                        f.data.empty() ? 0 : f.data[0]);
        }
    }
}

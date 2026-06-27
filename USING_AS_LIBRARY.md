# Using shm_bridge as a library

Both packages are now importable/linkable from your own code, not just runnable as
nodes. There are working examples in each package (`example_usage`).

---

## Python — `import shm_bridge_python`

Already on the Python path after `colcon build` + `source install/setup.bash`.

```python
from shm_bridge_python import StreamReader, adapt, shm_paths
from shm_bridge_python import shm_contract as C

# read any stream, type-agnostic — returns an ndarray (FLAT) or a ROS msg (CDR)
rd = StreamReader("camera__color__image_raw")
obj, hdr = rd.read_frame()        # obj: np.ndarray or deserialized message
print(hdr["seq"], hdr["data_size"], type(obj))

# turn any ROS message into a SHM payload (FLAT zero-copy or CDR)
a = adapt(my_msg, "sensor_msgs/msg/Image", topic="/cam")
print(a.encoding, a.width, a.height, a.dtype)
```

Run the example:
```bash
ros2 run shm_bridge_python example_usage write   # terminal 1
ros2 run shm_bridge_python example_usage read    # terminal 2
```
Source: `src/shm_bridge_python/shm_bridge_python/example_usage.py`

---

## C++ — `#include <shm_bridge_cpp/shm_bridge.hpp>`

Built as a real shared library: `libshm_bridge_cpp.so`, with headers + an ament
export so other packages can `find_package` it.

```cpp
#include <shm_bridge_cpp/shm_bridge.hpp>

// writer
shm_bridge::Writer w("rgb", 1920*1080*4);
w.write_flat(ptr, len, width, height, channels,
             shm_bridge::DType::U8, "sensor_msgs/msg/Image", "/rgb");
w.write_cdr(serialized_ptr, serialized_len, "tf2_msgs/msg/TFMessage");

// reader (type-agnostic)
shm_bridge::Reader r("rgb");
shm_bridge::Frame f;
if (r.read(f)) {
    // f.data, f.width, f.height, f.channels, f.dtype, f.encoding,
    // f.type_name, f.seq, f.timestamp_ns
}
```

Run the example:
```bash
ros2 run shm_bridge_cpp example_usage write   # terminal 1
ros2 run shm_bridge_cpp example_usage read    # terminal 2  (use `stdbuf -oL` if piping)
```
Source: `src/shm_bridge_cpp/examples/example_usage.cpp`

### Linking it from ANOTHER package
In that package's `CMakeLists.txt`:
```cmake
find_package(shm_bridge_cpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)
target_link_libraries(my_node shm_bridge_cpp::shm_bridge_cpp)
```
and in `package.xml`: `<depend>shm_bridge_cpp</depend>`.

What gets installed (so downstream `find_package` works):
- `lib/libshm_bridge_cpp.so`
- `include/shm_bridge_cpp/{shm_bridge,shm_contract,flat_adapter}.hpp`
- `share/shm_bridge_cpp/cmake/shm_bridge_cppConfig.cmake` (+ Targets)

---

## Is this architecture-dependent (x86_64 / arm64)?

**Source: portable. Compiled artifacts: per-arch.**

- The **C++ source/headers** and **Python code** are architecture-independent —
  POSIX `mmap`, `std::atomic`, standard ROS 2. They compile and run on x86_64,
  arm64 (Jetson), etc.
- The built **`libshm_bridge_cpp.so` is a binary for the CPU you built it on**
  (here: `ELF 64-bit ... x86-64`). To run on arm64, just `colcon build` on the
  arm64 machine — it produces an arm64 `.so`. This is normal ROS 2 behavior; you
  ship source, not binaries.
- **Endianness:** the on-disk header is little-endian (Python `struct '<'`, C++
  fixed offsets) and the FLAT path memcpys raw buffers. x86_64 and arm64 are both
  little-endian, so frames and the `.so` are compatible across them. Only a
  big-endian CPU (not used in robotics) would break it — and since SHM is
  same-host, producer and consumer are always the same arch anyway.

**Bottom line:** write once, rebuild per target, safe on x86_64 + arm64.
```

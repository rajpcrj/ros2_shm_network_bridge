# Build & install

There are **two ways** to consume this library:

- **A. As a ROS 2 / colcon package** — for ROS nodes. You source the workspace and
  `find_package(shm_bridge_cpp)`.
- **B. As a plain system library** — for ANY C++ program, no ROS needed. You copy
  the `.so` + headers to `/usr/local` and just `#include <>` + `-lshm_bridge_cpp`.

---

## Prerequisites
- Ubuntu 22.04, ROS 2 Humble (only needed for path A and the ROS example nodes)
- `g++` ≥ 11 (C++17), `cmake`, `colcon`
- Python 3.10+ with `numpy` (for the Python examples)

---

## A. Build as a ROS 2 package (colcon)

```bash
cd /path/to/ros2_shm_fanout
source /opt/ros/humble/setup.bash
colcon build --packages-select shm_bridge_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

This produces:
- `install/shm_bridge_cpp/lib/libshm_bridge_cpp.so`
- `install/shm_bridge_cpp/include/shm_bridge_cpp/*.hpp`

Use it from another ament package's `CMakeLists.txt`:
```cmake
find_package(shm_bridge_cpp REQUIRED)
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp sensor_msgs)
target_link_libraries(my_node shm_bridge_cpp::shm_bridge_cpp)
```
and in code: `#include <shm_bridge_cpp/shm_bridge.hpp>`.

Run the bundled examples:
```bash
ros2 run shm_bridge_cpp example_usage write     # terminal 1
ros2 run shm_bridge_cpp example_usage read      # terminal 2
```

---

## B. Install system-wide (no ROS needed to USE it)

After building once (step A, or even just the library), run the installer:

```bash
./install_lib.sh                      # -> /usr/local  (uses sudo for the copy)
# or, user-local without sudo:
PREFIX=$HOME/.local ./install_lib.sh
```

It copies `libshm_bridge_cpp.so` → `$PREFIX/lib` and the headers →
`$PREFIX/include/shm_bridge_cpp/`, then runs `ldconfig`. Now ANY program builds with
just:

```bash
g++ -std=c++17 examples/cpp/read_cpp.cpp  -o read_cpp  -lshm_bridge_cpp -lpthread
g++ -std=c++17 examples/cpp/write_cpp.cpp -o write_cpp -lshm_bridge_cpp -lpthread
./write_cpp &      # terminal 1
./read_cpp         # terminal 2  -> prints seq/size/first-byte as frames arrive
```

`#include <shm_bridge_cpp/shm_bridge.hpp>` resolves because the headers are on the
default include path; `-lshm_bridge_cpp` resolves because the `.so` is on the
default library path (`ldconfig` cache).

> If you used a custom `PREFIX`, add `-I$PREFIX/include -L$PREFIX/lib` at compile
> time and `export LD_LIBRARY_PATH=$PREFIX/lib` at run time.

Uninstall: `./install_lib.sh --uninstall`.

---

## Building the network + ROS examples standalone
`ros2_to_shm.cpp` / `shm_to_ros2.cpp` need rclcpp + sensor_msgs, so build them
inside the colcon workspace (path A). `shm_to_network.cpp`, `write_cpp.cpp`,
`read_cpp.cpp` are pure POSIX/library and build with plain `g++` after step B:

```bash
g++ -std=c++17 examples/cpp/shm_to_network.cpp -o shm_to_network -lshm_bridge_cpp -lpthread
```

---

## Rebuilding the `.so` on a new machine / architecture
The **source** is architecture-independent; the **built `.so` is per-arch**. On a
new target (e.g. ARM64 Jetson) just rebuild: `colcon build --packages-select
shm_bridge_cpp` then `./install_lib.sh`. The `/dev/shm` byte layout is
little-endian and works across x86_64 and arm64.

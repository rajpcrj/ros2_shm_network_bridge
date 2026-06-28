# The modular core — "is there an interface card?"

**Yes.** The system is already split into a **ROS-agnostic core** and thin
**adapters**. The core knows nothing about ROS, DDS, or message types — it only
moves bytes through shared memory. Each adapter is the "interface card" that plugs a
specific world (ROS 2, a network socket, OpenCV, …) into that core.

```
            ┌─────────────────────────────────────────────────────────────┐
            │   CORE  (libshm_bridge_cpp.so)   — NO ROS, NO DDS, NO deps    │
            │   shm_bridge::Writer / Reader / Frame                         │
            │   seqlock + futex + the /dev/shm byte contract                │
            └───────────────▲───────────────────────────▲─────────────────┘
                            │ #include <shm_bridge_cpp/shm_bridge.hpp>
        ┌───────────────────┴──────┐     ┌──────────────┴───────────────┐
        │  ADAPTER: ros2_to_shm    │     │  ADAPTER: shm_to_udp /        │
        │  #include <rclcpp/...>   │     │            shm_to_network     │
        │  ROS topic  -> core      │     │  core -> UDP socket           │
        └──────────────────────────┘     └───────────────────────────────┘
        (also: shm_to_ros2, an OpenCV adapter, a Python adapter, …)
```

## Proof the core is ROS-free
`src/shm_bridge_cpp/src/shm_bridge.cpp` (the entire core implementation) includes
**only** its own headers + the C++ standard library + POSIX — no `rclcpp`, no
`sensor_msgs`, no DDS. You can verify:
```bash
grep -E '#include' src/shm_bridge_cpp/src/shm_bridge.cpp
#   -> shm_bridge.hpp, shm_contract.hpp, shm_futex.hpp, <chrono>, <cstring>, ...
```
That is why `libshm_bridge_cpp.so` links with just `-lshm_bridge_cpp -lpthread` and
runs in programs that have never sourced ROS (see `examples/cpp/{write,read}_cpp.cpp`).

## What lives in the core (execution only)
The core's job is **mechanism, not policy**:
- allocate/map the `/dev/shm` files for a stream,
- the seqlock write/read protocol (torn-frame safety),
- the futex wake/wait (O(1) fan-out),
- the 64-byte header contract + FLAT/CDR framing.

It does **not** know what a "ROS Image" is, how to serialize CDR, or how to open a
socket. Those are adapter concerns.

## What an adapter does (policy / glue)
An adapter is a small `.cpp` that `#include`s both the core and one external API,
and translates between them. The pattern is always the same:

**Ingest adapter** (something → SHM): subscribe/receive in the foreign API, call
`Writer::write_flat(...)`.
```cpp
#include <shm_bridge_cpp/shm_bridge.hpp>   // the core
#include <rclcpp/rclcpp.hpp>               // the "interface card" for ROS 2
#include <sensor_msgs/msg/image.hpp>
// ... on each ROS message: writer.write_flat(m->data.data(), m->data.size(), ...);
```

**Egress adapter** (SHM → something): `Reader::wait_and_read(...)`, then push into
the foreign API (publish a ROS topic, `sendto()` a socket, `cv::imshow`, …).

Because the foreign include is *only* in the adapter, swapping it is a file-level
change: want a ROS 1 bridge? write `ros1_to_shm.cpp` that `#include`s `ros/ros.h`
instead of `rclcpp` — the core is untouched. Want a GStreamer source? write
`gst_to_shm.cpp`. The core never changes.

## The adapters that ship today
| adapter | foreign API it cards-in | direction |
|---|---|---|
| `examples/cpp/ros2_to_shm.cpp` | `rclcpp` + `sensor_msgs` | ROS 2 topic → SHM |
| `examples/cpp/shm_to_ros2.cpp` | `rclcpp` + `sensor_msgs` | SHM → ROS 2 topic |
| `examples/cpp/shm_to_udp.cpp` | POSIX sockets | SHM → UDP server (raw memory) |
| `examples/cpp/shm_to_network.cpp` | POSIX sockets | SHM → UDP (fragmented frames) |
| `examples/python/*.py` | Python `mmap` + numpy | both, via the byte contract |

## Build-time modularity
- The **core** builds with zero ROS deps and can be installed standalone
  (`install_lib.sh`) for use by *any* program.
- The **ROS adapters** are the only targets that `ament_target_dependencies(... rclcpp
  sensor_msgs)`. If you build the library outside ROS, you simply don't build those
  targets — the core still works.

## Writing your own adapter (recipe)
1. `#include <shm_bridge_cpp/shm_bridge.hpp>` and your foreign API header.
2. For ingest: create a `Writer(stream, max_bytes)`; on each input call `write_flat`
   (or `write_cdr` for serialized bytes).
3. For egress: create a `Reader(stream)`; loop on `wait_and_read(f, timeout)` and
   forward `f` to your output.
4. Link `shm_bridge_cpp` (+ your foreign API's libs). Done — the core is reused as-is.

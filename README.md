# ros2_shm_network_bridge

A generic, **type-agnostic** ROS 2 ↔ shared-memory transport. A *writer* subscribes
to any topic and publishes the message into `/dev/shm` under a lock-free **seqlock**;
a *reader* — which knows nothing about the message type at compile time —
reconstructs it on the other side. Same-host, low-latency, zero-copy where it counts.

It is a **transport, not a viewer**: the reader rebuilds the data (numpy array or ROS
message); rendering is out of scope. See [PROMPT.md](PROMPT.md) for the full design.

## Packages

| Package | Type | What |
|---|---|---|
| `shm_bridge_cpp` | ament_cmake | C++ nodes + reusable `libshm_bridge_cpp.so` library |
| `shm_bridge_python` | ament_python | Python nodes + importable `shm_bridge_python` module |

## How it works (1 paragraph)

Per stream there are three files in `/dev/shm`: a fixed 64-byte **binary header**
(seqlock + size + shape, updated every frame), the raw **frame** buffer, and a JSON
**recipe** (full type name + decode info, written only when structure changes). The
writer picks **FLAT** (zero-copy numeric buffer, for the 25 heavy image/cloud/tensor
types) or **CDR** (`serialize_message`, universal — covers all 330 topic types). The
reader reads the seqlock, copies the frame, re-reads the seqlock, and retries if it
changed (torn-frame detection). Full byte contract: [PROMPT.md](PROMPT.md) §4.

## Requirements

- ROS 2 **Humble** (other distros likely work; built/tested on Humble)
- A C++17 compiler, CMake ≥ 3.8
- Python deps: `numpy` (required). `flask` + `orjson` only for the legacy web/UDP
  viewers.

Install ROS deps from the workspace root:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Quick start

```bash
# bridge one topic (type is auto-detected from the graph — only pass the topic):
ros2 run shm_bridge_python ros2_to_shm --ros-args -p topic:=/camera/color/image_raw

# or bridge EVERY topic, one isolated process each:
ros2 run shm_bridge_python ros2_to_shm_all

# read it back (stream name = topic with '/' -> '__'):
ros2 run shm_bridge_python generic_shm_reader \
  --ros-args -p "streams:=[camera__color__image_raw]"
```

C++ equivalents: `ros2 run shm_bridge_cpp ros2_to_shm` (universal CDR) and
`ros2 run shm_bridge_cpp ros2_to_shm_flat` (typed zero-copy FLAT + CDR fallback).

## Use it as a library

Python:
```python
from shm_bridge_python import StreamReader, adapt, shm_paths
```
C++:
```cpp
#include <shm_bridge_cpp/shm_bridge.hpp>   // shm_bridge::Writer / Reader
```
Details + a downstream `find_package` snippet: [USING_AS_LIBRARY.md](USING_AS_LIBRARY.md).
Runnable demos: `ros2 run {shm_bridge_cpp,shm_bridge_python} example_usage write|read`.

## Architecture portability

Source is arch-independent (POSIX `mmap`, `std::atomic`); the compiled `.so` is
per-arch — just rebuild on each target. The on-disk format is little-endian, safe on
x86_64 and arm64. See [USING_AS_LIBRARY.md](USING_AS_LIBRARY.md#is-this-architecture-dependent-x86_64--arm64).

## Documentation

- [PROMPT.md](PROMPT.md) — the full design spec and byte contract
- [HOW_IT_WAS_BUILT.md](HOW_IT_WAS_BUILT.md) — how/why it was built, phase by phase
- [USING_AS_LIBRARY.md](USING_AS_LIBRARY.md) — importing/linking from your own code
- `src/message_type/*.txt` — classification of all installed message types
  (heavy/FLAT vs CDR, topic vs service-only)

## License

Apache-2.0 — see [LICENSE](LICENSE).

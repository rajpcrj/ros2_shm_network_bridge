# ros2_shm_network_bridge

A generic, **type-agnostic** ROS 2 ↔ shared-memory transport. A *writer* subscribes
to any topic and publishes the message into `/dev/shm` under a lock-free **seqlock**;
any number of *readers* on the same machine map the same bytes and wake on a single
**futex** (≈0 % CPU while idle). One write, one copy in RAM regardless of reader
count, O(1) fan-out — versus DDS's per-subscriber serialize-and-copy.

It is a **transport, not a viewer**: the reader hands back the raw bytes / a numpy
array / a rebuilt ROS message; rendering is out of scope.

```
   producer ──► /dev/shm/<stream>_{header,frame,recipe.json} ──► many readers (C++ / Python)
                          ▲ seqlock + futex                         └─► optional: shm_to_ros2, shm_to_network
```

---

## TL;DR — get started in 60 seconds

```bash
# 1. build the library
source /opt/ros/humble/setup.bash
colcon build --packages-select shm_bridge_cpp
source install/setup.bash

# 2. (optional) install system-wide so ANY C++ program can use it without ROS
./install_lib.sh                       # -> /usr/local  (sudo for the copy)

# 3. run the pure-C++ demo (no ROS needed)
g++ -std=c++17 examples/cpp/write_cpp.cpp -o /tmp/w -lshm_bridge_cpp -lpthread
g++ -std=c++17 examples/cpp/read_cpp.cpp  -o /tmp/r -lshm_bridge_cpp -lpthread
/tmp/w &        # writer
/tmp/r          # reader -> prints seq / size / first byte as frames arrive
```

Then in your own code:
```cpp
#include <shm_bridge_cpp/shm_bridge.hpp>
shm_bridge::Writer w("rgb", 1920*1080*4);
w.write_flat(ptr, len, W, H, 3, shm_bridge::DType::U8, "sensor_msgs/msg/Image");

shm_bridge::Reader r("rgb");
shm_bridge::Frame f;
r.wait_and_read(f);    // blocks ~0% CPU until a new frame, then fills f
```

---

## What you can do with it (all the recipes)

| Goal | Tool | Doc |
|---|---|---|
| Write to SHM from C++ | `examples/cpp/write_cpp.cpp` | [docs/02_cpp_api.md](docs/02_cpp_api.md) |
| Read from SHM in C++ | `examples/cpp/read_cpp.cpp` | [docs/02_cpp_api.md](docs/02_cpp_api.md) |
| Bridge a ROS 2 topic → SHM | `examples/cpp/ros2_to_shm.cpp` | [docs/02_cpp_api.md](docs/02_cpp_api.md) |
| Bridge SHM → a ROS 2 topic | `examples/cpp/shm_to_ros2.cpp` | [docs/02_cpp_api.md](docs/02_cpp_api.md) |
| Send SHM over the network (fire-and-forget, lowest latency) | `examples/cpp/shm_to_network.cpp` | [docs/04_network_transport.md](docs/04_network_transport.md) |
| Serve RAW SHM memory from a UDP **server** (many clients) | `examples/cpp/shm_to_udp.cpp` | [docs/04_network_transport.md](docs/04_network_transport.md) |
| Receive the network stream → SHM / subscribe to the server | `examples/python/{network_to_shm,udp_client}.py` | [docs/04_network_transport.md](docs/04_network_transport.md) |
| Understand the modular core / write your own adapter | — | [docs/06_modular_core.md](docs/06_modular_core.md) |
| Write/Read in Python | `examples/python/{write_py,read_py}.py` | [docs/05_python_and_cpp_interop.md](docs/05_python_and_cpp_interop.md) |
| Use C++ `.so` from Python | pybind11 (Option 2) | [docs/05_python_and_cpp_interop.md](docs/05_python_and_cpp_interop.md) |
| Install `.so`+headers to `/usr/local` | `./install_lib.sh` | [docs/03_build_and_install.md](docs/03_build_and_install.md) |

After `colcon build`, the C++ examples are runnable as ROS nodes too:
```bash
ros2 run shm_bridge_cpp ex_ros2_to_shm  --ros-args -p topic:=/camera/image_raw -p stream:=rgb
ros2 run shm_bridge_cpp ex_shm_to_ros2  --ros-args -p stream:=rgb -p topic:=/from_shm
ros2 run shm_bridge_cpp ex_write_cpp    # / ex_read_cpp / ex_shm_to_network
```

---

## Can Python use the C++ `.so`? (short answer)

**You don't need to** — and that's the point. Python and C++ talk through the **same
byte-exact `/dev/shm` contract**, so a C++ writer is read by a Python reader and
vice-versa **with zero glue** (verified both directions). If you specifically want
Python to call the C++ `Writer`/`Reader` classes (e.g. for the futex blocking),
wrap them with pybind11 — full guide and a copy-paste binding in
[docs/05_python_and_cpp_interop.md](docs/05_python_and_cpp_interop.md).

```bash
# proof: Python writes, C++ reads (same frames)
python3 examples/python/write_py.py demo &
/tmp/r          # the C++ reader from the TL;DR prints Python's frames
```

---

## Install as a system library (no ROS needed to USE it)

```bash
./install_lib.sh                 # /usr/local (default)
PREFIX=$HOME/.local ./install_lib.sh    # user-local, no sudo
./install_lib.sh --uninstall
```
Copies `libshm_bridge_cpp.so` → `$PREFIX/lib` and headers → `$PREFIX/include/shm_bridge_cpp/`,
runs `ldconfig`. Then any program builds with just `-lshm_bridge_cpp` and
`#include <shm_bridge_cpp/shm_bridge.hpp>`. Details: [docs/03_build_and_install.md](docs/03_build_and_install.md).

---

## Packages & layout

| Path | What |
|---|---|
| `src/shm_bridge_cpp/` | C++ nodes + the reusable `libshm_bridge_cpp.so` |
| `src/shm_bridge_python/` | Python package (same contract, ROS nodes, helpers) |
| `examples/cpp/`, `examples/python/` | copy-paste starter programs (this set) |
| `docs/` | in-depth guides (architecture, API, build, network, interop) |
| `install_lib.sh` | install `.so` + headers system-wide |

---

## How it works (1 paragraph)

Per stream, three files in `/dev/shm`: a fixed **64-byte binary header** (seqlock +
size + shape, updated every frame), the raw **frame** buffer, and a JSON **recipe**
(full type name + decode info). The writer picks **FLAT** (raw numeric buffer, for
heavy image/cloud/tensor types — reader reconstructs with no deserialize) or **CDR**
(`serialize_message`, universal). The writer never blocks: `seq++` to odd (writing),
copy payload, `seq++` to even (stable); a reader reads-body-between-two-seq-reads and
retries on a tear. After each publish one `FUTEX_WAKE` releases all waiting readers.
Full design: [docs/01_architecture.md](docs/01_architecture.md) and [PROMPT.md](PROMPT.md) §4.

---

## Performance & comparison (honest)

> **Note (naming):** during development this system is referred to simply as **"the
> bridge"** throughout the code, docs, and graphs. That is a working name — when this
> is deployed/published on the internet it may be renamed. Wherever you see
> "bridge", read it as "this `/dev/shm` transport".

Benchmarked head-to-head against **FastDDS** (data-sharing), **CycloneDDS**
(+iceoryx), and **ros2_shm_msgs**, following **Apex.AI `performance_test`**
methodology. Full methodology, image types/sizes, frame counts, and all graphs are
in **[test_runs/README.md](test_runs/README.md)**; raw data in
[test_runs/data/sweep.csv](test_runs/data/sweep.csv). 12-core laptop, 1–8 MiB
image frames, 30 Hz, K=4 runs, mean ± std-dev.

### CPU vs subscribers — the headline (O(1)-ish vs O(n))
![CPU vs subscribers, 1 MiB](test_runs/graphs/cpu_vs_subs_1m.png)

The bridge's CPU is **nearly flat** (~1.2 %/subscriber); every DDS transport climbs
**~8× steeper** (~10 %/sub). At **N=64**: bridge raw = **78 % of one core** vs
FastDDS/ros2_shm_msgs **~620–670 %** (~6–7 cores). The futex (no busy-poll) is why.

### Latency (p50) vs subscribers — the crossover
![Latency vs subscribers, 1 MiB](test_runs/graphs/latency_vs_subs_1m.png)

FastDDS-loaned wins at low N; the curves **cross near N≈16**, after which the bridge
leads because DDS becomes CPU-starved.

### Latency vs payload size (N=1) — where DDS-loaned wins
![Latency vs size, N=1](test_runs/graphs/latency_vs_size_n1.png)

FastDDS-loaned is **flat in size** (true zero-copy pointer handoff). The bridge's
read **copies** the bytes, so its latency is **O(size)**. This is stated honestly —
the bridge does **not** win single-subscriber latency.

### Delivered rate vs subscribers — integrity under load
![FPS vs subscribers, 1 MiB](test_runs/graphs/fps_vs_subs_1m.png)

The bridge holds ~29.8 fps / ~0 % loss to N=64; DDS drops to 13–25 fps when
CPU-bound, and **CycloneDDS crashes at N=32** (iceoryx pool exhaustion).

**Use it for** high-rate, large-payload, **same-machine** fan-out to many readers.
**Not** as a network transport on its own (see [docs/04](docs/04_network_transport.md)),
and not when single-reader latency on small messages is the only thing you care about.

---

## Requirements
- ROS 2 **Humble** (only for the ROS nodes; the core library + pure examples need no ROS)
- C++17 compiler, CMake ≥ 3.8 · Python 3.10+ with `numpy`
- `rosdep install --from-paths src --ignore-src -r -y` for ROS deps

## Architecture portability
Source is arch-independent (POSIX `mmap`, `std::atomic`); the compiled `.so` is
per-arch — rebuild on each target. On-disk format is little-endian (x86_64 + arm64).

## More docs
- [docs/01_architecture.md](docs/01_architecture.md) — seqlock, futex, FLAT vs CDR
- [docs/02_cpp_api.md](docs/02_cpp_api.md) — full Writer/Reader reference
- [docs/03_build_and_install.md](docs/03_build_and_install.md) — colcon + system install
- [docs/04_network_transport.md](docs/04_network_transport.md) — UDP, the `shm_to_udp` server, wire format
- [docs/05_python_and_cpp_interop.md](docs/05_python_and_cpp_interop.md) — the interop question, 3 options
- [docs/06_modular_core.md](docs/06_modular_core.md) — the ROS-agnostic core + how to write adapters
- [test_runs/README.md](test_runs/README.md) — benchmark methodology, image ladder, all graphs
- [PROMPT.md](PROMPT.md) · [HOW_IT_WAS_BUILT.md](HOW_IT_WAS_BUILT.md) · [USING_AS_LIBRARY.md](USING_AS_LIBRARY.md)

## License
Apache-2.0 — see [LICENSE](LICENSE).

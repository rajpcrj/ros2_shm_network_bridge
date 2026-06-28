# ros2_shm_fanout

> **A same-host shared-memory transport for high-fan-out perception.**

### The problem I hit
I was building a multi-model perception pipeline — one camera frame feeding several
models at once (object detection, segmentation, depth, pose, classification). On
constrained compute, ROS 2's default transport was the bottleneck: it **serializes and
copies the frame per subscriber**, so every model I added cost more CPU and more
memory, and under load the pipeline started **dropping frames**. The transport, not
the models, was the wall. So I built a transport designed for exactly this shape of
workload: **one producer, many hungry consumers, large frames, on a single machine.**

### What it is
A **type-agnostic shared-memory transport.** The writer puts **one copy** of each
frame into `/dev/shm` and signals readers with a **single futex wake**; every reader
**maps the same physical pages** and reads its own view. No per-subscriber
serialization, no per-subscriber copy, no DDS discovery/QoS machinery, no busy-polling
(idle readers sleep on a futex at ~0 % CPU). Two paths: a **FLAT zero-copy** path for
heavy numeric types (Image, PointCloud2, tensor-shaped arrays) and a **CDR fallback**
covering all ~330 ROS message types.

### Works beyond ROS
A deliberate design point, not an accident. Because the FLAT path writes **raw frame
bytes** into `/dev/shm` with a small binary header + a JSON recipe — **not** a
ROS-serialized blob — a consumer doesn't have to be a ROS node to read it. **Any
process that can `mmap` a file can consume the stream**: a non-ROS Python process in a
separate conda env, a CUDA program, a model server outside the ROS graph. That
mattered for my pipeline, where some consumers lived outside ROS entirely and the
standard transport couldn't reach them without bridging overhead.

### The findings
Benchmarked against the three production zero-copy transports — **FastDDS
data-sharing, CycloneDDS+iceoryx, ros2_shm_msgs** — same payloads, **K=4 runs with
variance**, whole-system (pid-set) CPU accounting. Full method + graphs:
[test_runs/](test_runs/README.md).

- **~8× lower CPU per consumer.** At **64 subscribers** on a 1 MiB stream the bridge
  used **~78 % of one core**; the DDS zero-copy paths used **~620–670 %** (~6 cores).
  The advantage is **structural** — the bridge's CPU rises **~1.2 %/subscriber** vs
  DDS's **~10 %/subscriber** — and it widens with subscriber count. At a realistic
  perception fan-out (**4–8 consumers**) it's already **~10–16× leaner**. *(This is
  lower CPU, not lower latency — at a single subscriber DDS-loaned can win latency;
  see [Performance](#performance--comparison).)*
- **Flat RAM.** Memory is **constant in the number of subscribers** (one physical
  copy, mapped by all) — RAM was unchanged from 1 to 64 consumers. Genuinely **O(1)**
  in subscriber count.
- **Full-rate delivery under load.** The bridge held **~30 fps with ~0 % loss out to
  64 subscribers** at every payload size. The DDS paths fell behind under CPU pressure
  (down to **13–25 fps**), **CycloneDDS crashed at 32 subscribers** (iceoryx pool
  exhaustion), and **ros2_shm_msgs segfaulted on an 8 MiB frame at a single
  subscriber**.

> **Scope:** the win is **CPU + memory + delivery integrity at fan-out** on a
> **single machine**, not single-subscriber latency and not a cross-machine transport.
> See [when NOT to use it](#so-why-is-it-good-for-this-use-case) and the
> [benchmark caveats](#how-the-benchmarking-was-performed) (intra-process, one-way
> latency). Numbers reproduce from [test_runs/data/sweep.csv](test_runs/data/sweep.csv).

---

A generic, **type-agnostic** ROS 2 ↔ shared-memory transport. A *writer* subscribes
to any topic and publishes the message into `/dev/shm` under a lock-free **seqlock**;
any number of *readers* on the same machine map the same bytes and wake on a single
**futex** (≈0 % CPU while idle). One write, one copy in RAM regardless of reader
count (RAM is O(1) in subscribers), and the writer's publish is one `FUTEX_WAKE` no
matter how many readers — versus DDS's serialize-and-copy *per subscriber*. (Total
delivery CPU is still O(N) readers, but with a ~8× smaller constant; see
[the analysis](#why-this-beats-plain-ros-2-here--dds-path-vs-the-bypass).)

It is a **transport, not a viewer**: the reader hands back the raw bytes / a numpy
array / a rebuilt ROS message; rendering is out of scope.

![Overview: one write into shared memory, many zero-copy readers](docs/diagrams/overview.png)

---

## Index

- [Quickstart](#quickstart) — terse, copy-paste
- [Quickstart elaborated](#quickstart-elaborated)
  - [The three scenarios](#the-three-scenarios) — A: ROS↔ROS · B: no-ROS · C: ROS→no-ROS
  - [The same three scenarios in Python](#the-same-three-scenarios-in-python)
- [Common use cases](#common-use-cases) — pick your task:
  - [1. One camera → many consumers on one PC](#uc1-one-camera--many-consumers-on-one-pc)
  - [2. Feed a non-ROS program (inference / GUI / logger)](#uc2-feed-a-non-ros-program-inference--gui--logger)
  - [3. Send ROS topics to another computer](#uc3-send-ros-topics-to-another-computer)
  - [4. Stream raw frames to many network clients (UDP server)](#uc4-stream-raw-frames-to-many-network-clients-udp-server)
  - [5. Bridge between C++ and Python](#uc5-bridge-between-c-and-python)
  - [6. Embed the library in your own app](#uc6-embed-the-library-in-your-own-app)
  - [7. Reproduce / extend the benchmarks](#uc7-reproduce--extend-the-benchmarks)
- [Reference: every example & where to read more](#reference-every-example--where-to-read-more)
- [Supported message types](#supported-message-types) — all 330 topic types (25 FLAT + 305 CDR)
- [Why this beats plain ROS 2 here — DDS path vs the bypass](#why-this-beats-plain-ros-2-here--dds-path-vs-the-bypass)
- [Packages & layout](#packages--layout)
- [How it works (1 paragraph)](#how-it-works-1-paragraph)
- [Performance & comparison](#performance--comparison) — graphs, test machine
- [How the benchmarking was performed](#how-the-benchmarking-was-performed) — method, files, why intra-process
- [Requirements](#requirements) · [Architecture portability](#architecture-portability)
- [More docs](#more-docs) · [License](#license)
- Deep docs: [architecture](docs/01_architecture.md) · [C++ API](docs/02_cpp_api.md) ·
  [build/install](docs/03_build_and_install.md) · [network](docs/04_network_transport.md) ·
  [Python↔C++](docs/05_python_and_cpp_interop.md) · [modular core](docs/06_modular_core.md) ·
  [benchmarks](test_runs/README.md) · [examples](examples/README.md) ·
  [end-to-end pipeline](examples/end_to_end/README.md) · [prebuilt lib](prebuilt/README.md)

---

## Supported message types

The bridge is **type-agnostic and covers every ROS 2 topic message type** — on this
install (ROS Humble) that is **330 topic types**, via two encodings the reader
dispatches automatically:

| Encoding | Count | What | Reconstructed as |
|---|---:|---|---|
| **FLAT** (zero-copy) | **25** | heavy numeric buffers (image / cloud / grid / tensor) | numpy `ndarray` (zero-copy view) |
| **CDR** (universal) | **305** | every other topic type — structs, nested, variable-length | rebuilt ROS message object |
| **Total** | **330** | all topic message types | — |

> **Adding a new type costs nothing.** CDR needs no per-type code — `serialize_message`
> / `deserialize_message` handle all 305 generically; you just point a writer at the
> topic. FLAT is an opt-in fast path with a small per-type extractor. The reader never
> changes. (Service-only `srv/*_Request|Response.msg` types — 286 of them — are not
> topic-publishable, so they are out of scope by definition.)

**CDR streams are meant for ROS 2 consumers.** A CDR payload is the message's
ROS-serialized form, so the natural consumer is another **ROS 2 node/package** that
deserializes it straight back into a real ROS message (`deserialize_message`) — no
type-specific code needed. **FLAT** streams, by contrast, are raw frame bytes and can
be read by *any* `mmap`-capable consumer, ROS or not (a non-ROS Python process, a CUDA
program, a model server).

<details>
<summary><b>The 25 FLAT (zero-copy) types</b></summary>

```
sensor_msgs/msg/Image                 sensor_msgs/msg/CompressedImage
sensor_msgs/msg/PointCloud2           sensor_msgs/msg/PointCloud
sensor_msgs/msg/LaserScan             sensor_msgs/msg/MultiEchoLaserScan
stereo_msgs/msg/DisparityImage        nav_msgs/msg/OccupancyGrid
map_msgs/msg/OccupancyGridUpdate      octomap_msgs/msg/Octomap
visualization_msgs/msg/MeshFile       rmw_dds_common/msg/Gid
ros_gz_interfaces/msg/Dataframe       ros_gz_interfaces/msg/Float32Array
std_msgs/msg/ByteMultiArray           std_msgs/msg/UInt8MultiArray
std_msgs/msg/Int8MultiArray           std_msgs/msg/UInt16MultiArray
std_msgs/msg/Int16MultiArray          std_msgs/msg/UInt32MultiArray
std_msgs/msg/Int32MultiArray          std_msgs/msg/UInt64MultiArray
std_msgs/msg/Int64MultiArray          std_msgs/msg/Float32MultiArray
std_msgs/msg/Float64MultiArray
```
Full lists: [HEAVY_FLAT.txt](src/message_type/HEAVY_FLAT.txt) ·
[LIGHT_CDR.txt](src/message_type/LIGHT_CDR.txt) (the 305) ·
[GENERAL_TOPIC.txt](src/message_type/GENERAL_TOPIC.txt) (all 330) ·
[SERVICE_ONLY.txt](src/message_type/SERVICE_ONLY.txt) (286, out of scope).
Regenerate for your own install: `python3 src/message_type/generate_index.py`.

> C++ note: `ros2_to_shm_flat` provides typed FLAT for these except
> `UInt64/Int64MultiArray` and `MultiEchoLaserScan`, which fall back to CDR (still
> fully supported). The Python adapter handles all 25 as FLAT.
</details>

---

## Quickstart

You know ROS 2, shared memory, and `g++`. Here's the whole thing.

**Library API** (`#include <shm_bridge_cpp/shm_bridge.hpp>`, link `-lshm_bridge_cpp -lpthread`):
```cpp
shm_bridge::Writer w("rgb", 1920*1080*4);
w.write_flat(ptr, len, W, H, 3, shm_bridge::DType::U8, "sensor_msgs/msg/Image");  // 1 copy in
shm_bridge::Reader r("rgb"); shm_bridge::Frame f;
r.wait_and_read(f);   // futex wait ~0% CPU, then f.data / f.width / f.seq ...
```
Lock-free seqlock writer + one `FUTEX_WAKE` per publish (O(1) wake), one RAM copy
regardless of reader count. Streams are `/dev/shm/<name>_{header,frame,recipe.json}`.

**Use the lib with zero build** (a clone ships a prebuilt x86-64 `.so` in `prebuilt/`):
```bash
git clone https://github.com/rajpcrj/ros2_shm_fanout.git
cd ros2_shm_fanout
g++ -std=c++17 my.cpp -o my -Iprebuilt/include -Lprebuilt/lib -lshm_bridge_cpp -lpthread
LD_LIBRARY_PATH=$PWD/prebuilt/lib ./my
# or install it system-wide, then no -I/-L needed anywhere:
./install_lib.sh        # cp .so+headers -> /usr/local + ldconfig (auto-uses prebuilt/)
```

**Build from source** (ROS nodes + benchmarks):
```bash
git clone https://github.com/rajpcrj/ros2_shm_fanout.git
cd ros2_shm_fanout
source /opt/ros/humble/setup.bash && colcon build --packages-select shm_bridge_cpp
source install/setup.bash
```

**Run anything:**
```bash
# pure C++ (no ROS), bundled sample image:
ros2 run shm_bridge_cpp ex_write_cpp & ros2 run shm_bridge_cpp ex_read_cpp
# ROS topic -> SHM -> ROS topic:
ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args -p topic:=/image -p stream:=rgb
ros2 run shm_bridge_cpp ex_shm_to_ros2 --ros-args -p stream:=rgb -p topic:=/from_shm
# all topics over UDP to another box (4-stage pipeline, see examples/end_to_end/):
ros2 run shm_bridge_cpp e2e_1_ros2_to_shm        # machine A
ros2 run shm_bridge_cpp e2e_2_shm_to_udp --host <B> --port 7000
ros2 run shm_bridge_cpp e2e_3_udp_to_shm --port 7000   # machine B
ros2 run shm_bridge_cpp e2e_4_shm_to_ros2
```

**Python** talks the same `/dev/shm` contract (no `.so` linking; reads/writes C++
streams): `export PYTHONPATH=src/shm_bridge_python:$PYTHONPATH` then
`python3 examples/python/{write_py,read_py}.py demo`. To call the C++ classes from
Python instead, use the pybind11 wrapper in [docs/05](docs/05_python_and_cpp_interop.md).

**Perf (1 MiB, K=4, vs FastDDS/CycloneDDS/ros2_shm_msgs):** ~1.2 %/sub CPU vs DDS
~10 %/sub (≈8× at N=64); only transport holding ~30 fps / 0 % loss to 64 subs; DDS
*loaned* wins single-subscriber latency. Full data + graphs: [test_runs/](test_runs/README.md).

---

## Quickstart elaborated

Same thing as the Quickstart above, but explained from scratch for someone new to
ROS 2 / shared memory / `g++`. New to all this? Read these 6 lines first:

- **What is this?** A way to move a picture (or any data) from one program to another
  on the **same computer**, extremely fast, by putting it in a special shared area of
  RAM called `/dev/shm`. One program (the **writer**) puts data in; one or many other
  programs (the **readers**) take it out. Nobody copies the data more than once.
- **What is a "stream"?** Just a name (like `demo`). The writer and readers must use
  the **same name** to find each other. Behind the scenes it becomes three files in
  `/dev/shm/` (e.g. `demo_header`, `demo_frame`, `demo_recipe.json`).
- **What is a `.so` file?** `libshm_bridge_cpp.so` is the compiled **library** — the
  actual machine code that does the shared-memory work. Your program "borrows" it.
- **What is a header (`.hpp`)?** A text file that tells your C++ program *what
  functions exist* in the library, so you can call them. You `#include` it.
- **What is ROS 2?** A robotics framework. Programs publish data on named "topics".
  Some examples here connect to ROS 2; some don't need it at all.
- **What does `g++` do?** It turns your `.cpp` source code into a runnable program.

### One-time setup (do this once)

```bash
source /opt/ros/humble/setup.bash          # turn on ROS 2 in this terminal
colcon build --packages-select shm_bridge_cpp   # compile the library (makes the .so)
source install/setup.bash                  # tell this terminal where the build is
```

### Two ways to USE the library in your own C++ programs

Whenever you compile your own `.cpp` that uses this library, you pick **one** of these:

**Way 1 — install it system-wide, then forget about paths** (cleanest):
```bash
./install_lib.sh        # copies the .so to /usr/local/lib and headers to /usr/local/include
                        # (this is literally a `cp` to /usr/local + `ldconfig`; uses sudo)
# now ANY program, anywhere, builds with just:
g++ -std=c++17 my_program.cpp -o my_program -lshm_bridge_cpp -lpthread
```
`-lshm_bridge_cpp` means "link the shm_bridge_cpp library". Because `install_lib.sh`
put it in a standard folder, the compiler finds it automatically — no paths needed.

**Way 2 — don't install anything; point g++ at the files directly**:
```bash
g++ -std=c++17 my_program.cpp -o my_program \
    -I /full/path/to/ros2_shm_fanout/src/shm_bridge_cpp/include \
    -L /full/path/to/ros2_shm_fanout/install/shm_bridge_cpp/lib \
    -lshm_bridge_cpp -lpthread
# and at RUN time, tell the program where the .so lives:
export LD_LIBRARY_PATH=/full/path/to/ros2_shm_fanout/install/shm_bridge_cpp/lib
./my_program
```
`-I` = "look here for headers (`.hpp`)". `-L` = "look here for libraries (`.so`)".
`LD_LIBRARY_PATH` = "look here for the `.so` when the program *runs*".

> Inside every example `.cpp` you'll see `#include <shm_bridge_cpp/shm_bridge.hpp>` at
> the top — that one line is how the source file "includes the library". Way 1 or Way
> 2 just tells the compiler where that header and its `.so` actually are.

---

## The three scenarios

Pick the one that matches your situation. Each is self-contained.

### A. ROS 2 on BOTH sides (writer reads a ROS topic, reader publishes a ROS topic)

Use this when you already have data flowing as ROS 2 topics (e.g. a camera driver)
and want a second ROS program to receive it through the fast `/dev/shm` path.

**Step 1 — find a real image topic.** Plug in a camera (or play a bag), then list
what's available:
```bash
ros2 topic list                       # shows all topics, e.g. /camera/image_raw
ros2 topic info /camera/image_raw     # confirm its type is sensor_msgs/msg/Image
ros2 topic hz /camera/image_raw       # confirm images are actually arriving
```
No camera? Start a fake image publisher in another terminal:
```bash
ros2 run image_tools cam2image --ros-args -p burger_mode:=true   # publishes /image
```

**Step 2 — writer: ROS topic → shared memory.** (terminal 1)
```bash
ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args \
     -p topic:=/image -p stream:=rgb
# "subscribe to the ROS topic /image, put each frame into the shm stream named rgb"
```

**Step 3 — reader: shared memory → a new ROS topic.** (terminal 2)
```bash
ros2 run shm_bridge_cpp ex_shm_to_ros2 --ros-args \
     -p stream:=rgb -p topic:=/from_shm
# "read the shm stream rgb, re-publish it as the ROS topic /from_shm"
```

**Step 4 — check it worked.** (terminal 3)
```bash
ros2 topic hz /from_shm        # should show ~the same rate as /image
```

---

### B. NO ROS 2 at all (pure C++, uses the bundled sample image)

Use this when you just want two plain programs to share data — no robotics framework.
This scenario ships a **sample image** so you don't need a camera.

The sample lives at `examples/sample_data/sample_640x480_rgb.bin` (a 640×480 image
stored as raw red-green-blue bytes; there's also a viewable `.png` next to it).

**Step 1 — compile the two example programs** (using Way 1 above; do `./install_lib.sh`
first):
```bash
cd examples/cpp
g++ -std=c++17 write_cpp.cpp -o write_cpp -lshm_bridge_cpp -lpthread
g++ -std=c++17 read_cpp.cpp  -o read_cpp  -lshm_bridge_cpp -lpthread
```

**Step 2 — run the writer** (terminal 1). It loads the sample image and publishes it
~30×/second into the stream `demo`:
```bash
cd examples
cpp/write_cpp sample_data/sample_640x480_rgb.bin 640 480
# -> [write_cpp] loaded ... -> publishing to /dev/shm/demo_*
```

**Step 3 — run the reader** (terminal 2). It prints one line per frame it receives:
```bash
cd examples
cpp/read_cpp
# -> seq=92  640x480x3  921600 bytes  type=sensor_msgs/msg/Image  first byte=0
```
That's the sample image flowing writer → shared memory → reader, with no ROS involved.

---

### C. ROS 2 on the WRITER, NO ROS 2 on the READER (the common real case)

Use this when ROS produces the data, but the consumer is a plain C++/Python program
(an inference engine, a logger, a GUI) that you **don't** want to burden with ROS.

**Step 1 — writer (ROS):** same as scenario A step 2. (terminal 1)
```bash
ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args -p topic:=/image -p stream:=rgb
```

**Step 2 — reader (NO ROS):** the plain `read_cpp` from scenario B, just pointed at
the same stream name `rgb`. (terminal 2)
```bash
# read_cpp attaches to the stream "demo" by default; for scenario C either change
# the stream name in read_cpp.cpp to "rgb" and recompile, or run the ROS writer with
# -p stream:=demo so the names match. (The reader needs NO ROS to run.)
cd examples && cpp/read_cpp
```
The ROS world fills the shared memory; a program that has never heard of ROS reads it.
That is the whole point — **the consumer pays zero ROS cost.**

---

## The same three scenarios in Python

Python does **not** need to link the `.so`. It talks to the *same* `/dev/shm` files
using the bundled `shm_bridge_python` module — so Python and C++ freely read each
other's data (verified both directions). First make Python find the module:
```bash
export PYTHONPATH=/full/path/to/ros2_shm_fanout/src/shm_bridge_python:$PYTHONPATH
```

- **A. ROS 2 both sides (Python):** use the ROS Python nodes:
  ```bash
  ros2 run shm_bridge_python ros2_to_shm --ros-args -p topic:=/image     # writer
  ros2 run shm_bridge_python generic_shm_reader --ros-args -p "streams:=[image]"  # reader
  ```
- **B. No ROS (Python, sample image):**
  ```bash
  python3 examples/python/write_py.py demo     # terminal 1: publishes into stream "demo"
  python3 examples/python/read_py.py  demo     # terminal 2: prints each frame
  ```
- **C. ROS writer, plain-Python reader (the mix-and-match magic):**
  ```bash
  # C++ OR ROS writes the stream...
  cd examples && cpp/write_cpp sample_data/sample_640x480_rgb.bin 640 480 &
  # ...and plain Python reads it, no ROS:
  python3 python/read_py.py demo
  # Equally: python3 write_py.py demo  &  then the C++ cpp/read_cpp reads it.
  ```

> **Can Python use the C++ `.so` directly?** You don't need to — the shared `/dev/shm`
> contract is the interface. If you *want* Python to call the C++ classes (for the
> 0%-CPU futex wait), there's a ready-made pybind11 wrapper in
> [docs/05_python_and_cpp_interop.md](docs/05_python_and_cpp_interop.md).

---

## Common use cases

Pick the task that matches yours. Each points at the exact program(s) and doc.

### UC1. One camera → many consumers on one PC
*You have a high-rate camera/lidar and several programs that all need it, and DDS is
burning CPU copying to each.* Bridge the topic once; every consumer maps the same
bytes — **RAM is O(1)** in the number of consumers (one shared copy) and **CPU is
O(N) but ~8× lower slope** than DDS (~1.2 %/sub vs ~10 %/sub).
```bash
ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args -p topic:=/camera/image_raw -p stream:=cam
# each consumer: shm_bridge::Reader r("cam"); r.wait_and_read(f);   (C++ or Python)
```
→ scenario A, [docs/02](docs/02_cpp_api.md). Why it scales: [Performance](#performance--comparison).

### UC2. Feed a non-ROS program (inference / GUI / logger)
*ROS produces the data, but the consumer is a plain C++/Python app you don't want to
drag ROS into.* The reader needs **zero** ROS.
```bash
ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args -p topic:=/image -p stream:=rgb
# non-ROS reader (links only the .so, or pure Python):
g++ -std=c++17 my_infer.cpp -o my_infer -lshm_bridge_cpp -lpthread   # #include <shm_bridge_cpp/shm_bridge.hpp>
```
→ scenario C, [docs/06 modular core](docs/06_modular_core.md).

### UC3. Send ROS topics to another computer
*Get some/all topics from machine A to machine B over the LAN, transparently.* The
4-stage pipeline re-creates the topics on B with original names + types.
```bash
# machine A:  e2e_1_ros2_to_shm  +  e2e_2_shm_to_udp --host <B> --port 7000
# machine B:  e2e_3_udp_to_shm --port 7000  +  e2e_4_shm_to_ros2
```
→ [examples/end_to_end/](examples/end_to_end/README.md) (incl. ssh / two-machine setup),
[docs/04](docs/04_network_transport.md). Localhost smoke test: `bash examples/end_to_end/test_localhost.sh`.

### UC4. Stream raw frames to many network clients (UDP server)
*Several remote viewers want the same stream, on demand.* Run a UDP server that ships
the raw `/dev/shm` bytes to any client that subscribes.
```bash
ros2 run shm_bridge_cpp ex_shm_to_udp --stream rgb --port 6000     # server
python3 examples/python/udp_client.py --host <server> --port 6000  # each client
```
→ `examples/cpp/shm_to_udp.cpp`, [docs/04](docs/04_network_transport.md).

### UC5. Bridge between C++ and Python
*A C++ producer and a Python consumer (or vice-versa) on one machine.* They share the
same `/dev/shm` contract — no bindings, no copies beyond the mmap. Verified both ways.
```bash
cpp/write_cpp sample_data/sample_640x480_rgb.bin 640 480 &   # C++ writes
python3 python/read_py.py demo                              # Python reads
```
→ [docs/05 Python↔C++](docs/05_python_and_cpp_interop.md) (and the pybind11 option to
call the C++ classes from Python).

### UC6. Embed the library in your own app
*You just want the fast SHM transport inside your codebase.* Use the prebuilt `.so`
(ships in the repo) — no colcon, no ROS.
```bash
./install_lib.sh        # cp prebuilt .so+headers -> /usr/local (single source of truth)
g++ -std=c++17 app.cpp -o app -lshm_bridge_cpp -lpthread
# OR without installing: -Iprebuilt/include -Lprebuilt/lib  + LD_LIBRARY_PATH=prebuilt/lib
```
→ [docs/03 build/install](docs/03_build_and_install.md), [prebuilt/](prebuilt/README.md).

### UC7. Reproduce / extend the benchmarks
*You want to re-run the transport comparison or add a transport.* The harness +
methodology + graphs are all here.
```bash
python3 tests/run_test5.py          # the grid that produced the graphs
python3 test_runs/make_graphs.py    # re-render graphs from data/sweep.csv
```
→ [test_runs/](test_runs/README.md), and the methodology docs in `src/shm_bridge_cpp/benchmark/`.

---

## Reference: every example & where to read more

| I want to… | File | Beginner? | Deep doc |
|---|---|---|---|
| Write/read in pure C++ (sample image) | `examples/cpp/{write,read}_cpp.cpp` | scenario B | [docs/02](docs/02_cpp_api.md) |
| ROS topic → SHM | `examples/cpp/ros2_to_shm.cpp` | scenario A/C | [docs/02](docs/02_cpp_api.md) |
| SHM → ROS topic | `examples/cpp/shm_to_ros2.cpp` | scenario A | [docs/02](docs/02_cpp_api.md) |
| Send SHM over the network (one host) | `examples/cpp/shm_to_network.cpp` | — | [docs/04](docs/04_network_transport.md) |
| Serve SHM from a UDP **server** (many clients) | `examples/cpp/shm_to_udp.cpp` | — | [docs/04](docs/04_network_transport.md) |
| Receive the network stream | `examples/python/{network_to_shm,udp_client}.py` | — | [docs/04](docs/04_network_transport.md) |
| Write/read in Python | `examples/python/{write,read}_py.py` | scenario B | [docs/05](docs/05_python_and_cpp_interop.md) |
| Install the `.so` system-wide | `./install_lib.sh` | Way 1 above | [docs/03](docs/03_build_and_install.md) |
| Understand the design / write your own adapter | — | — | [docs/01](docs/01_architecture.md), [docs/06](docs/06_modular_core.md) |

---

## Why this beats plain ROS 2 here — DDS path vs the bypass

To see *why* this is the right tool for same-machine, large-data, many-consumer
fan-out, you have to see what a normal ROS 2 subscription actually does under the hood.

### What a normal `ros2` publish→subscribe does (the DDS path)

When you `create_subscription` in ROS 2, you are not talking to the publisher
directly. ROS 2 is an API on top of a **DDS** middleware (FastDDS, CycloneDDS, …) via
the **RMW** layer. A single message travels like this:

![Normal ROS 2 path: every message goes through the DDS middleware](docs/diagrams/dds_path.png)

The costs that matter for big, frequent data:

- **Serialize + deserialize every message** (steps 3 & 7) — CPU proportional to
  payload size, paid **once per publish AND once per subscriber**.
- **A copy (often several) per subscriber** (steps 4, 6, 8) — default loopback DDS
  copies the bytes through the kernel **for each subscriber**, so RAM traffic and CPU
  grow **O(n) in the number of subscribers**.
- **Discovery + QoS machinery** — participants discover each other and run liveliness/
  history bookkeeping continuously.
- **Executor scheduling** — each subscriber's callback is dispatched by an executor;
  with many subscribers that's many wakeups and context switches.

DDS *can* do better with **shared-memory QoS / loaned messages** (FastDDS
data-sharing, CycloneDDS+iceoryx), and those are excellent — but they still run the
full RTPS stack (discovery, QoS, per-endpoint delivery), still serialize on the
*normal* path, and in our benchmark still cost **~10 % of a core per subscriber**.
For a single subscriber they can even *beat* this bridge on latency. Their weakness is
**CPU at fan-out**, which is exactly the case this tool targets.

### How this bridge bypasses the DDS layer

This bridge does **not** use DDS, RMW, RTPS, discovery, or per-subscriber delivery at
all. It writes the payload **once** into a POSIX shared-memory buffer that every reader
**maps directly**. The path collapses to:

![This bridge bypasses DDS: one write, one wake, readers map the same bytes](docs/diagrams/bridge_path.png)

What got removed, and why it wins **for this use case**:

| DDS step | This bridge | effect |
|---|---|---|
| serialize / deserialize (FLAT path) | **none** — raw bytes are the payload | no per-msg, per-sub CPU for big arrays |
| per-subscriber kernel copies | **one** copy into shared memory, readers map it | **RAM is O(1) in #subscribers** (shared physical pages); **CPU is O(N) with a low constant** (~1.2 %/sub) vs DDS's ~10 %/sub |
| discovery / QoS / RTPS | **none** — just a named file in `/dev/shm` | no background middleware overhead |
| executor wakeups per sub | **one** `FUTEX_WAKE` syscall by the writer | the writer's *notify* is O(1); each reader still does its own wake-up + read, so total delivery work is O(N) — just a much smaller constant than DDS |
| busy-poll to notice new data | `FUTEX_WAIT` (sleeps) | idle readers use **~0 % CPU** |

The mechanics are real and verifiable in the code:
[`write_flat` does a single `memcpy`](src/shm_bridge_cpp/src/shm_bridge.cpp) then
`notify()`; [`notify()` is one `FUTEX_WAKE`](src/shm_bridge_cpp/include/shm_bridge_cpp/shm_futex.hpp);
the [`Reader` `mmap`s the same frame](src/shm_bridge_cpp/src/shm_bridge.cpp) and
`FUTEX_WAIT`s when idle. The lock-free **seqlock** (odd=writing, even=stable, reader
retries on a tear) is what lets the writer never block on readers.

### So: why is it good for THIS use case?

For **same-machine, high-rate, large-payload, many-consumer** delivery, the bridge
replaces DDS's *serialize-and-copy-per-subscriber* with *one write + one futex wake +
shared pages*. **RAM stays O(1)** in the number of subscribers (one physical copy,
mapped by all). **CPU is still O(N)** — each reader must wake and read its own copy,
so you can't deliver to N consumers for free — **but the per-subscriber constant is
~8× smaller than DDS** (~1.2 %/sub vs ~10 %/sub). That is precisely UC1/UC2 above.
Measured: ≈**8× lower CPU at 64 subscribers** (78 % of one core vs ~620 %), RAM flat
across N=1→64, full rate with ~0 % loss while DDS falls behind
([Performance](#performance--comparison)).

> **On complexity, precisely:** delivering to N readers is inherently Ω(N) work — N
> readers each do a `FUTEX_WAIT` + read. So the bridge is **O(N) in CPU**, not O(1);
> its win is a *much lower slope*, not a lower complexity class. Only **RAM** is
> genuinely O(1) in subscriber count. Don't let the strong RAM claim smuggle in a
> false CPU claim — the real numbers (flat RAM, 8×-lower-slope CPU) are the result.

**When NOT to reach for it:** across machines (it's local-only — use the
[network examples](docs/04_network_transport.md) or DDS), for tiny messages where the
copy savings are negligible, or when you specifically need DDS features (reliability
QoS, history, lifecycle, cross-vendor interop). The bridge is a **specialist for the
fan-out case**, not a general DDS replacement.

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

## Performance & comparison

> **Note (naming):** during development this system is referred to simply as **"the
> bridge"** throughout the code, docs, and graphs. That is a working name — when this
> is deployed/published on the internet it may be renamed. Wherever you see
> "bridge", read it as "this `/dev/shm` transport".

Benchmarked head-to-head against **FastDDS** (data-sharing), **CycloneDDS**
(+iceoryx), and **ros2_shm_msgs**, following **Apex.AI `performance_test`**
methodology. Full methodology, image types/sizes, frame counts, and all graphs are
in **[test_runs/README.md](test_runs/README.md)**; raw data in
[test_runs/data/sweep.csv](test_runs/data/sweep.csv). 8-core / 12-thread laptop
(i5-13420H), 1–8 MiB image frames, 30 Hz, K=4 runs, mean ± std-dev.

### CPU vs subscribers — the headline (same O(N), ~8× lower slope)
![CPU vs subscribers, 1 MiB](test_runs/graphs/cpu_vs_subs_1m.png)

Both the bridge and DDS rise with subscriber count (delivery is inherently O(N)); the
bridge just has a **far lower slope**. The bridge's CPU climbs ~1.2 %/subscriber; every DDS transport climbs
**~8× steeper** (~10 %/sub). At **N=64**: bridge raw = **78 % of one core** vs
FastDDS/ros2_shm_msgs **~620–670 %** (~6–7 cores). The futex (no busy-poll) is why.

### Latency (p50) vs subscribers — the crossover
![Latency vs subscribers, 1 MiB](test_runs/graphs/latency_vs_subs_1m.png)

FastDDS-loaned wins at low N; the curves **cross near N≈16**, after which the bridge
leads because DDS becomes CPU-starved.

### Latency vs payload size (N=1) — where DDS-loaned wins
![Latency vs size, N=1](test_runs/graphs/latency_vs_size_n1.png)

FastDDS-loaned is **flat in size** (true zero-copy pointer handoff). The bridge's
read **copies** the bytes, so its latency is **O(size)** — the bridge does **not**
win single-subscriber latency.

### Delivered rate vs subscribers — integrity under load
![FPS vs subscribers, 1 MiB](test_runs/graphs/fps_vs_subs_1m.png)

The bridge holds ~29.8 fps / ~0 % loss to N=64; DDS drops to 13–25 fps when
CPU-bound, and **CycloneDDS crashes at N=32** (iceoryx pool exhaustion).

**Use it for** high-rate, large-payload, **same-machine** fan-out to many readers.
**Not** as a network transport on its own (see [docs/04](docs/04_network_transport.md)),
and not when single-reader latency on small messages is the only thing you care about.

### Test machine / environment

All benchmark numbers and graphs above were produced on this machine
(`uname -a`, `lscpu`, `free -h`, etc.):

| | |
|---|---|
| **Host / kernel** | `Linux raj-Nitro-ANV15-51 6.8.0-90-generic #91~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC x86_64` |
| **OS** | Ubuntu 22.04.5 LTS |
| **CPU** | 13th Gen Intel® Core™ i5-13420H — 8 cores / **12 threads** (HT), 1 socket, 400–4600 MHz |
| **Caches** | L1d 320 KiB · L1i 384 KiB · L2 7 MiB · L3 12 MiB |
| **RAM** | 23 GiB total |
| **GPU** | Intel (a7a8, integrated) + NVIDIA (28a1, discrete) — *not used by the benchmark* |
| **CPU governor** | `powersave` (could not set `performance` — no sudo on the rig) |
| **Turbo** | enabled (`intel_pstate/no_turbo = 0`) |
| **ROS 2** | Humble (`/opt/ros/humble`) |
| **Toolchain** | gcc 11.4.0 (C++17, `-O3`) · Python 3.10.12 · matplotlib 3.10.7 |
| **CPU pinning / isolation** | none (`isolcpus` not set); a desktop session was running |

> Because the governor was `powersave` and CPUs were not isolated, **absolute**
> latency is conservative and carries extra jitter — but this affects all four
> transports **equally**, so the **relative** comparison stays fair. The pid-set CPU
> metric is immune to background desktop activity by construction (see
> [test_runs/README.md](test_runs/README.md)). Reproduce the exact environment
> snapshot any time with: `uname -a && lscpu && free -h`.

---

## How the benchmarking was performed

The numbers and graphs above come from a **custom harness** that follows Apex.AI
`performance_test` methodology (fixed-rate absolute schedule to avoid coordinated
omission, percentiles, 20 % warm-up trim, K repeated runs → mean ± std-dev, 95 %
CPU/RAM health gates, and a **pid-set CPU** metric that also counts the iox-roudi
daemon so DDS gets no free ride). The same byte-identical payloads, rate, and stats
code run for all four transports — only the publish/subscribe mechanism is swapped.

- **Benchmark source code (in this repo):** `src/shm_bridge_cpp/benchmark/` (the
  `test4_benchmarks/` binaries + `bench_common.hpp`).
- **Raw data + all graphs + the full image ladder (1–8 MiB) + methodology (in this
  repo):** [test_runs/](test_runs/README.md). Re-render the graphs any time with
  `python3 test_runs/make_graphs.py`.
- **Driver that orchestrated the grid:** `run_test5.py` (lives in the surrounding
  benchmarking workspace, not committed inside this library repo) →
  `K=4 NLIST="1 4 16 32 64" python3 run_test5.py`. Its committed output (`sweep.csv`)
  is in `test_runs/data/`, so the results here are fully reproducible from the data.
- **The deeper "why" docs:**
  [BENCHMARKING.md](src/shm_bridge_cpp/benchmark/BENCHMARKING.md) (why a custom
  harness, what's better/worse, what was skipped),
  [INTRA_VS_INTER_PROCESS.md](src/shm_bridge_cpp/benchmark/INTRA_VS_INTER_PROCESS.md),
  and [ROUND_TRIP.md](src/shm_bridge_cpp/benchmark/ROUND_TRIP.md).

### Why intra-process (threads), not inter-process (separate processes)?

Real ROS 2 deployments run nodes as **separate processes**, and the headline DDS
numbers people usually cite come from that **inter-process** setup. Our harness instead
ran the publisher and all subscribers as **threads in one process** (intra-process).
We chose this deliberately, and it's important to understand the trade-off.

The reason is **isolation of the variable.** With everything in one process, the only
thing that differs between transports is the publish/subscribe path itself — there is
no inter-process scheduling noise, no separate executors fighting for cores, and the
publisher and subscriber share one `CLOCK_MONOTONIC`, which makes a direct one-way
latency measurement valid without a round-trip relay or clock synchronization. That
gives a clean, controlled, apples-to-apples **relative** comparison of the four
transports under identical conditions.

The cost of that choice is that intra-process **removes costs DDS pays in
production** — participant discovery, per-process executor overhead, and cross-process
wakeups. In other words, our setup is, if anything, **generous to DDS**: in a real
inter-process deployment DDS would pay *more*, so the bridge's CPU/scalability
advantage is expected to be **at least as large, probably larger**, than what we
report. The flip side is that our **absolute** milliseconds are lower than a standard
inter-process round-trip benchmark would show, so they should **not** be compared to
published `performance_test` numbers — only to each other.

We did not skip inter-process out of convenience to flatter the bridge; we skipped it
to isolate the mechanism, and we're explicit that the realistic inter-process run
(plus a round-trip latency path) is the planned follow-up. The full reasoning, with
diagrams of what work appears in each mode, is in
[INTRA_VS_INTER_PROCESS.md](src/shm_bridge_cpp/benchmark/INTRA_VS_INTER_PROCESS.md).

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

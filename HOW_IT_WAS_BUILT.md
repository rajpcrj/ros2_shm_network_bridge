# HOW_IT_WAS_BUILT.md — the construction story

This explains **how** `ros2_shm_fanout` was built — the reasoning, the
order, the dead-ends ruled out, and how each piece fits. It is the "why/how"
companion to `PROMPT.md` (the spec) and is not a run guide.

---

## 0. Starting point

The repo began as loose scripts: a C++ ROS 2 package (`shm_bridge`) and a `python/`
folder with three standalone scripts, all moving RealSense D455 RGB + depth out of
ROS topics into shared memory (or UDP) and viewing it. Not a git repo. The goal
grew, step by step, into a **generic type-agnostic ROS 2 → shared-memory transport**.

The build happened in distinct phases, each gated on a decision. Below is each
phase, what was decided, and why.

---

## 1. Phase 1 — restructure into two real ROS 2 packages

**Problem:** code was split as "C++ package" + "loose python/ scripts." Not a clean
ROS 2 workspace.

**What was done:**
- `src/shm_bridge` → `src/shm_bridge_cpp` (ament_cmake), `project()` and
  `package.xml` `<name>` renamed to `shm_bridge_cpp`.
- `python/` → `src/shm_bridge_python`, made a proper **ament_python** package
  (`setup.py`, `setup.cfg`, `resource/` marker, `__init__.py`). The 3 scripts
  became `console_scripts` entry points.

**Why:** so a single `colcon build` at the repo root builds *both*, and every node
runs via `ros2 run <pkg> <node>`. Confirmed with `colcon list` + `ros2 pkg
executables`.

**Also fixed while here:** the C++ package used OpenCV in CMake but never declared
it in `package.xml`; added `<depend>OpenCV</depend>` and `target_include_directories`
for OpenCV on both targets (the viewer was relying on headers being on a default
path — fragile).

---

## 2. Phase 2 — the hard design conversation (what is even possible)

Before writing the generic bridge, a long design discussion settled what "type
agnostic" can and cannot mean. This is the most important phase because it stopped
us building something impossible. The conclusions (captured fully in PROMPT.md §1):

- **Three separable concerns:** move bytes / encode-decode / render.
- **Reader can be type-agnostic; writer cannot.** The writer must know a message's
  layout to encode it; the reader just follows a recipe. Type knowledge concentrates
  in the writer + recipe, it doesn't vanish.
- **Rendering all types is impossible.** A `.msg` schema gives fields, never visual
  meaning. So the project scope became **reconstruct the message** (transport), not
  **render it** (viewer).
- Several user ideas were tested against reality and reshaped:
  - "decode instructions in the JSON" → yes, became the **recipe**.
  - "the type is a YAML/.msg file" → true but it describes structure, not rendering,
    and the YAML-echo form is far too slow for SHM.
  - "seqlock inside the JSON, addressable by memory" → contradiction (JSON is
    variable-length text, not atomically addressable) → became a **binary header**.

**Output of this phase:** `PROMPT.md`, the byte-exact spec everything else follows.

---

## 3. Phase 3 — the on-disk contract (the thing both languages obey)

The core invention. For each stream `NAME`, three files in `/dev/shm`:

| File | What | Update rate |
|---|---|---|
| `NAME_header` | 64-byte binary header, mmap'd, fixed offsets | every frame (atomic) |
| `NAME_frame` | raw payload (FLAT buffer or CDR bytes) | every frame |
| `NAME_recipe.json` | human-readable decode recipe | only on structural change |

The 64-byte header (offset 0 = `seq`, then `encoding_id`, `data_size`, `width`,
`height`, `channels`, `dtype_id`, `timestamp_ns`, `char[24] type_name`) is the fast
path. **`data_size` lives in the header so it can change every frame** without
rewriting the recipe — this is how varying payload sizes are handled at zero cost.

**Seqlock** (classic): writer bumps `seq` odd before writing, even after. Reader
reads seq → reads frame → re-reads seq; if it changed, the frame was torn → retry.
The counter had to be **binary and atomically addressable**, which is *why* it's in
the header and not the JSON.

Implemented as a single source of truth per language:
- `shm_contract.py` (struct pack/unpack, mmap helpers, atomic recipe write)
- `shm_contract.hpp` (`#pragma pack`, `static_assert(sizeof(Header)==64)`,
  `std::atomic` seq load/store with acquire/release).

---

## 4. Phase 4 — Python first (writer + reader)

Built Python first because `rclpy` introspection is far easier and let us nail the
recipe format before porting.

- **`generic_shm_writer.py`** — picks FLAT (zero-copy numeric, for Image/PointCloud2)
  vs CDR (`serialize_message`, universal) and writes header+frame+recipe under the
  seqlock.
- **`generic_shm_reader.py`** — type-agnostic. Implements the exact
  read/re-read/retry protocol; FLAT → `np.frombuffer().reshape()`, CDR →
  `deserialize_message(buf, get_message(type_name))`. **No message type is hardcoded.**

**Proven before touching C++** with an offline test (`scratchpad/test_contract.py`):
11/11 — FLAT round-trip (Image), CDR round-trip (TFMessage, a nested non-visual
type), and seqlock tear rejection (odd seq + changed seq both discarded).

**Why this gate mattered:** it confirmed `rclpy.serialization`,
`rosidl_runtime_py.utilities.get_message`, and the message packages all import and
work at runtime here (an earlier `ros2 interface list` had returned 0, which turned
out to be an unsourced-index quirk, not a real absence).

---

## 5. Phase 5 — port to C++, prove cross-language

The payoff test: the contract is only real if C++ and Python interoperate byte-for-byte.

- **`generic_shm_writer.cpp`** — uses `rclcpp::GenericSubscription` to subscribe to
  ANY type by name at runtime (no compile-time type dep) → CDR path.
- **`generic_shm_reader.cpp`** — type-agnostic reader, no ROS deps, pure contract.

**All four cross-language combinations verified live:**
- Python writer → C++ reader (CDR /tf)
- C++ writer → Python reader (CDR /tf, rebuilt a real TFMessage)
- Python writer FLAT Image → Python reader (ndarray, bytes identical)
- Python writer FLAT Image → C++ reader (5x4x3 dtype_id=0)

Seq values in every run were **even** (48, 50, 52…), confirming the seqlock.

---

## 6. Phase 6 — read every message type, classify them

The user asked to "read every .msg file." Built `src/message_type/generate_index.py`,
an **offline tool** (no ROS sourcing needed) that:
- copies every installed schema to `src/message_type/<pkg>/<Name>.msg`,
- writes `index.json` mapping `type_name → {encoding_hint, fields}`.

Numbers discovered on this machine (ROS Humble):
- **616** total `.msg` files, but that **includes service-embedded** `srv/*_Request.msg`
  / `*_Response.msg`. The honest split:
  - **330** pure topic messages (`msg/*.msg`) → `GENERAL_TOPIC.txt`
  - **286** service-only → `SERVICE_ONLY.txt`
- Then, by transport weight: **25 heavy/zero-copy** types (`HEAVY_FLAT.txt`) vs
  **305 everything-else** (`LIGHT_CDR.txt`). The 25 are exactly the perception/
  high-bandwidth carriers (Image, CompressedImage, PointCloud2, LaserScan,
  DisparityImage, OccupancyGrid, Octomap, all `*MultiArray` tensors, MeshFile, …).

**Key honesty point established here:** CDR needs **no per-type adapter** — it's
already universal. Per-type code only buys something on the **FLAT** fast path.

---

## 7. Phase 7 — the single adapter + per-stream-process nodes

The user wanted (a) one adapter function for all types, (b) lowest latency, with
each stream in its own OS process, (c) auto-detected type, (d) an `--all` mode.

**Latency reasoning (stated plainly to the user):** one process per stream removes
cross-stream interference and (in Python) escapes the GIL — worth doing. But the SHM
write path itself (`memcpy` + seqlock) was already near-optimal; processes don't make
the write faster, they isolate it. So: **one process per stream, single-threaded
inside.**

Built:
- **`adapter.py`** — single `adapt(msg, type, topic)` entry. Per-type FLAT
  extractors for all 25 heavy types (zero-copy `np.frombuffer` views, no intermediate
  allocation), CDR for the other 305. Returns a `structural_key` so the recipe is
  rewritten only when *structure* changes — not when `data_size` changes (that rides
  the header).
- **`ros2_to_shm.py`** — one topic per process. **Auto-resolves the type** from the
  graph (`get_topic_names_and_types`, the `ros2 topic type` equivalent); only `topic`
  is required. Stream name auto-derived `/a/b → a__b`, with the verbatim topic stored
  in the recipe so the reader never reverse-engineers it.
- **`ros2_to_shm_all.py`** — supervisor that dynamically watches the graph and
  **spawns one `ros2_to_shm` child process per topic**, reaping ones that vanish.
- **`ros2_to_shm.cpp`** — C++ mirror: one process per stream, auto-resolve type,
  universal CDR via `GenericSubscription`.

**Bug found and fixed here:** the header `type_name` is `char[24]`, which truncates
long names (`sensor_msgs/msg/JointState` = 26 chars → "JointSta"). Fix: the recipe
JSON carries the full name and the reader uses THAT for CDR reconstruction; the
header field is a hint only. (PROMPT.md §4.1 had anticipated this.)

Verified: auto-resolution + auto-naming + FLAT(Image)/CDR(JointState,Imu) round-trip;
`--all` spawned one isolated process per live topic; C++ `ros2_to_shm` auto-resolve →
Python reader rebuilt an Imu.

---

## 8. Phase 8 — full FLAT adapter in C++

The Python adapter had all 25 FLAT handlers; C++ only had Image/depth in the legacy
node. To match:

- **`flat_adapter.hpp`** — typed zero-copy extractors for the heavy types, each
  returning a pointer into the message buffer (no copy). Mirrors `adapter.py`.
- **`ros2_to_shm_flat.cpp`** — auto-resolves the type, and for a heavy type creates a
  **typed** subscription + FLAT extractor; for anything else falls back to
  `GenericSubscription`/CDR. Links the 7 extra message packages
  (std_msgs/nav_msgs/stereo_msgs/map_msgs/octomap_msgs/visualization_msgs/
  rmw_dds_common) plus sensor_msgs.

**Why C++ needed a separate node:** typed FLAT extraction requires the message class
at compile time, which is incompatible with `GenericSubscription`'s type-erased
bytes. So FLAT lives in a typed node; the universal CDR node stays generic.

**Caveats recorded:** `UInt64/Int64MultiArray` go CDR (the 8-entry dtype table has no
64-bit *integer* id), and `MultiEchoLaserScan` is genuinely nested so it falls back
to CDR.

Verified: C++ FLAT Image → Python reader ndarray (2,3,3) uint8; LaserScan → ndarray
float32 — both cross-language.

---

## 9. The throughline (why it ends up correct)

Every phase was **gated on a working test before moving on**, and every claim about
"agnostic" was checked against what information actually exists in the bytes:

- The **writer owns type knowledge** (encode); the **reader is generic** (decode by
  recipe). This asymmetry is enforced everywhere.
- **Two metadata channels at two rates:** fast binary header (per frame, incl.
  data_size + seqlock) and slow JSON recipe (on structural change, incl. the full
  type name). This is what gives both low latency and correctness.
- **CDR is the universal backstop** (any of the 330 topic types), **FLAT is the
  opt-in fast path** (the 25 heavy types, per-type extractors in both languages).
- **One process per stream** for isolation/low latency; `--all` is just a supervisor
  that fans that out across every topic.

The result: a single `colcon build` produces both packages; the same byte contract is
honored by Python and C++ in all directions; and adding a new stream is either "just
list the topic" (CDR) or "add one extractor" (FLAT) — the reader never changes.

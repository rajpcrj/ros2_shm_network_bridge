# PROMPT.md — Generic Type-Agnostic ROS 2 Shared-Memory Bridge

This file is the working specification for the next phase of `ros2_shm_network_bridge`.
It captures every decision made during design discussion, the constraints that are
**hard limits** (not laziness — actual information-theoretic walls), and the exact
on-disk formats so the C++ and Python sides stay byte-compatible.

Hand this file to an engineer (or an AI agent) and they should be able to build the
system without re-deriving the design.

> **STATUS: IMPLEMENTED & VERIFIED (2026-06-26).** All of §10 is done. See §12 for
> what was built and how to run it.

---

## 0. One-paragraph summary

Build a **generic, type-agnostic shared-memory transport** for ROS 2. A *writer* node
subscribes to any topic, serializes the incoming message into `/dev/shm`, and writes a
**decode recipe** (JSON) plus an **atomic seqlock** so that a *reader* — which knows
**nothing** about the message type at compile time — can reconstruct the original
message on the other side. This is a **transport, not a viewer**: the reader rebuilds
the data; it does not render it.

---

## 1. The mental model (read this before arguing with the constraints)

There are three separable concerns. Keep them separate:

| Concern | Who owns it | Can it be type-agnostic? |
|---|---|---|
| **Move the bytes** (SHM copy) | both | ✅ yes — bytes are bytes |
| **Encode / decode** (struct ⇄ bytes) | writer encodes, reader decodes | ✅ reader yes, writer no* |
| **Render** (bytes → picture) | (out of scope here) | ❌ no — needs human-supplied meaning |

\* The **writer cannot be type-agnostic**: to encode efficiently it must know the
message's schema (field layout). The **reader CAN be type-agnostic**: it just follows
the recipe the writer wrote. This asymmetry is the whole design. The type knowledge
does not disappear — it concentrates in the writer + the recipe.

### Hard limit (do not try to "fix" this)
- A `.msg` schema tells you the **fields**, never the **visual meaning**. `Image` and
  `PointCloud2` both expose `uint8[] data`; nothing in the schema says "draw the first
  as pixels, colormap the z-channel of the second." Rendering intent is human knowledge.
- Therefore: **"auto-convert all 616 types into pictures" is impossible.** ~600 of the
  616 types (`Bool`, `Twist`, `TFMessage`, `String`, …) have nothing to render.
- What **is** possible: **reconstruct the original message** for all 616 types. That is
  this project's scope.

---

## 2. Scope decisions (locked)

These were explicitly chosen during design. Do not silently re-litigate.

- **Reader job:** *Reconstruct only (pure transport).* Rebuild the message object/array
  from the recipe and stop. No rendering. (Display can be layered on separately later.)
- **Languages:** *Both C++ and Python.* Recommended build order: **Python first**
  (rclpy introspection is far easier), nail the recipe format, then port to C++.
- **Topic config:** *ROS parameters.* `rgb_topic` / `depth_topic` style params (defaults
  `/rgb`, `/depth_pcl`). Message **types** stay fixed per subscription — name-agnostic,
  not type-agnostic at subscription time. (A fully dynamic "subscribe to anything" mode
  is a later option; see §8.)
- **Sync scheme:** *Classic seqlock* (writer never blocks; reader retries on tear), with
  the **counter in a binary, mmap-addressable location** — NOT inside the JSON text.
- **Header layout:** *Binary header file per stream **+** keep the JSON recipe.* Binary
  header = fast atomic path; JSON = human-readable decode recipe + debugging.
- **Schema store:** Save all message schemas under `src/message_type/` for reference and
  for the writer/reader to consult at runtime.

---

## 3. Why the seqlock cannot live in the JSON (settled)

The reader's tear-detection protocol requires reading the counter **atomically** and
**without parsing**. JSON text is:
- variable-length (`"width":1280` vs `"width":640` shifts every later byte → no fixed
  offset), and
- rewritten whole each frame (`json.dump` / `ofstream`) → not atomic.

So the counter gets its own fixed-offset binary slot. The **decode recipe** (the part the
user wanted "in a text file describing how to decode") lives in JSON exactly as desired;
the **seqlock** is a separate tiny binary atomic. Two concerns, two files.

---

## 4. On-disk contract (C++ and Python MUST agree byte-for-byte)

For each stream `NAME` (e.g. `rgb`, `depth`), three artifacts in `/dev/shm`:

### 4.1 Binary header — `/dev/shm/NAME_header`
Fixed 64-byte little-endian layout, `mmap`-ed, addressable by offset:

```
offset  type        field         meaning
------  ----------  ------------  -------------------------------------------------
0       uint32      seq           SEQLOCK. even = stable, odd = write in progress.
                                  increments every write so any concurrent write is
                                  detectable (not just "caught in the act").
4       uint32      encoding_id   0 = FLAT (zero-copy numeric), 1 = CDR (serialized)
8       uint32      data_size     valid bytes in NAME_frame
12      uint32      width         (FLAT only; 0 otherwise)
16      uint32      height        (FLAT only; 0 otherwise)
20      uint32      channels      (FLAT only; 0 otherwise)
24      uint32      dtype_id      (FLAT only; see dtype table below)
28      uint32      reserved
32      uint64      timestamp_ns
40      char[24]    type_name     null-padded "pkg/msg/Name" (truncated if longer;
                                  full name always also in the JSON recipe)
```

`dtype_id` table (FLAT path): `0=uint8 1=int8 2=uint16 3=int16 4=uint32 5=int32
6=float32 7=float64`.

### 4.2 Frame buffer — `/dev/shm/NAME_frame`
Raw payload. For FLAT: the contiguous numeric buffer. For CDR: the serialized message
bytes. Size = `data_size` (header).

### 4.3 JSON recipe — `/dev/shm/NAME_recipe.json`
Human-readable decode recipe. Re-read by the reader only when dimensions change; the
binary header is the per-frame fast path. Example (FLAT):

```json
{
  "type_name": "sensor_msgs/msg/Image",
  "encoding": "flat",
  "dtype": "uint8",
  "shape": [720, 1280, 3],
  "data_size": 2764800,
  "field_map": {"data": {"offset": 0, "len": 2764800}},
  "timestamp_ns": 0
}
```

Example (CDR — universal path):

```json
{
  "type_name": "tf2_msgs/msg/TFMessage",
  "encoding": "cdr",
  "data_size": 412,
  "timestamp_ns": 0
}
```

---

## 5. Writer algorithm

```
on_message(msg, type_name):
    # 1. Decide encoding (the realistic version of "fastest way")
    if is_flat_numeric(type_name):        # Image, PointCloud2 blob, *MultiArray, etc.
        payload, shape, dtype = extract_contiguous_buffer(msg)
        encoding = FLAT
    else:                                  # everything else, all 616 covered
        payload = serialize_message(msg)   # rclpy.serialization / rclcpp CDR
        encoding = CDR

    # 2. SEQLOCK write (writer never blocks)
    seq = load(header.seq)
    store(header.seq, seq + 1)             # -> odd: "writing"
    release_fence()
    memcpy(NAME_frame, payload)
    fill_header_fields(encoding, data_size, shape, dtype, type_name, ts)
    write_json_recipe(...)                 # cheap; only must be self-consistent
    release_fence()
    store(header.seq, seq + 2)             # -> even: "stable"
```

`is_flat_numeric` is a **small explicit predicate**, not magic: a type qualifies for FLAT
if it is (or reduces to) one contiguous primitive array + scalar dimensions. Everything
else takes the CDR path. CDR is the correctness backstop that makes "all 616" true.

---

## 6. Reader algorithm (type-agnostic — this is the payoff)

This is exactly the protocol the user specified ("read seqlock; if good read the whole
file; re-read seqlock; if it changed it's corruption → retry"):

```
loop:
    s1 = atomic_load_acquire(header.seq)
    if s1 is odd:        # writer mid-write
        continue         # back off, retry
    read header fields
    copy NAME_frame[0:data_size]   # the "read the whole file" step
    maybe re-read JSON recipe if shape changed
    acquire_fence()
    s2 = atomic_load_acquire(header.seq)
    if s1 != s2:         # writer touched it during our read => torn => corruption
        continue         # discard, retry  (THIS IS CORRECT — confirmed)
    # s1 == s2 and even => frame is consistent
    if encoding == FLAT:
        obj = np.frombuffer(buf, dtype).reshape(shape)   # zero-copy reconstruct
    else:
        obj = deserialize_message(buf, get_message(type_name))  # universal
    deliver(obj)
```

The reader hardcodes **no message type**. It dispatches only on `encoding` (FLAT vs CDR),
`dtype`, and `shape` — all supplied by the writer. Adding new streams never touches the
reader as long as they use FLAT or CDR (they always do).

---

## 7. `src/message_type/` — the schema store

- Dump every installed schema so the tooling has an offline reference and the FLAT/CDR
  predicate can be precomputed/validated.
- Source of truth is the **local install**, not the internet: there is no canonical
  global list — any package (including yours) can define types. The local set is the
  authoritative set for this machine.
- This machine (ROS **Humble**) currently has **616 `.msg` files** across ~50 packages.
  NOTE: that 616 count **includes service-embedded `.msg` files** (`srv/*_Request.msg`,
  `srv/*_Response.msg`). Pure topic messages are the `msg/*.msg` subset.

Regenerate the list anytime with:

```bash
# all message-ish schema files (incl. srv-embedded)
find /opt/ros/humble/share -name '*.msg' | sort

# pure topic messages only, as pkg/msg/Name
find /opt/ros/humble/share -path '*/msg/*.msg' \
  | sed -E 's|.*/share/([^/]+)/msg/([^/]+)\.msg|\1/msg/\2|' | sort

# dump each schema's text
ros2 interface show <pkg>/msg/<Name>
```

Suggested layout: `src/message_type/<pkg>/<Name>.msg` mirroring the install tree, plus a
generated `index.json` mapping `type_name -> {encoding_hint: flat|cdr, fields: [...]}`.

---

## 8. Explicitly OUT of scope (and why)

- **Rendering / a generic viewer for all types** — impossible (see §1). Visual types can
  get a *separate* optional renderer later; the transport does not depend on it.
- **Fully dynamic "subscribe to any topic by name at runtime"** — possible later via
  `rclpy`'s generic subscription (`rclpy.node.Node.create_subscription` with a type
  looked up from the graph). Deferred; current scope keeps typed subscriptions config'd
  by parameter.
- **Cross-host** — SHM is local-only. The existing `ros2_udp_streamer.py` is the
  cross-host path and is unrelated to this design.

---

## 9. Build / run contract (must stay true)

- Single `colcon build` at repo root builds **both** packages (already verified):
  - `shm_bridge_cpp` (ament_cmake)
  - `shm_bridge_python` (ament_python)
- Run pattern:
  ```bash
  source /opt/ros/humble/setup.bash
  colcon build
  source install/setup.bash
  ros2 run shm_bridge_python <node>   # or shm_bridge_cpp
  ```
- Runtime pip deps (not rosdep keys): `orjson` (UDP streamer), `flask` (web viewer).
  `orjson` is currently NOT installed on this machine.

---

## 10. Definition of done

1. `src/message_type/` populated with all local schemas + generated `index.json`.
2. Python writer + reader implementing §4–§6, proven on `Image` (FLAT) and a non-visual
   type e.g. `TFMessage` or `JointState` (CDR) — reader reconstructs both with **zero**
   type-specific code.
3. Seqlock tear-detection demonstrably works (reader rejects torn frames under load).
4. C++ writer + reader ported to the **same** binary header + JSON recipe contract;
   cross-language proven: C++ writer ↔ Python reader and vice versa.
5. Single `colcon build` still green; both packages install all nodes.

---

## 11. Open questions to resolve before/while building

- FLAT predicate exact membership: confirm the list of types treated as zero-copy
  (start: `Image`, `PointCloud2`, `*MultiArray`; expand cautiously).
- Max buffer sizing per stream (current code assumes 1920×1080×3). Make it grow/cap
  cleanly instead of a hardcoded `ftruncate`.
- Whether to drop the legacy `*_meta.json` (`rgb_meta.json` etc.) once `*_recipe.json`
  exists, or keep both for backward compat.

---

## 12. What was built (implementation notes)

Verified on ROS **Humble**, single `colcon build` green, 9 executables installed.

### Files
- `src/message_type/generate_index.py` — dumps schemas + `index.json`. Produced
  **330 pure topic-message schemas** (the 616 figure includes srv-embedded `.msg`);
  index tags 38 flat-hint / 292 cdr. The hint is loose by design (the writer's real
  FLAT predicate is the narrow allowlist below).
- `src/shm_bridge_python/shm_bridge_python/shm_contract.py` — the byte layout
  (single source of truth). Atomic recipe writes via tmp+rename.
- `.../generic_shm_writer.py` — FLAT for `Image`/`PointCloud2`, CDR for everything
  else. Params: `streams=["name:topic:Type", ...]`, `max_bytes`.
- `.../generic_shm_reader.py` — type-agnostic; seqlock retry; FLAT→ndarray,
  CDR→`deserialize_message`. Params: `streams=["name", ...]`, `poll_hz`.
- `src/shm_bridge_cpp/include/shm_bridge_cpp/shm_contract.hpp` — mirrors the layout
  (`#pragma pack`, `static_assert(sizeof(Header)==64)`, `std::atomic` seq).
- `src/shm_bridge_cpp/src/generic_shm_writer.cpp` — uses `rclcpp::GenericSubscription`
  → CDR for **any** type at runtime, no compile-time type dep.
- `src/shm_bridge_cpp/src/generic_shm_reader.cpp` — type-agnostic reader (no ROS dep).

### FLAT predicate (Python writer, current membership)
`sensor_msgs/msg/Image`, `sensor_msgs/msg/PointCloud2`. Everything else → CDR.
Extend `FLAT_HANDLERS` in `generic_shm_writer.py` to add more.

### Cross-language verification (all passed)
- Python writer → C++ reader (CDR `/tf`): even seqs, reconstructed.
- C++ writer → Python reader (CDR `/tf`): rebuilt `TFMessage` object.
- Python writer FLAT `Image` → Python reader: ndarray `(4,5,3)` uint8, bytes identical.
- Python writer FLAT `Image` → C++ reader: `5x4x3 dtype_id=0`.
- Offline contract test (`test_contract.py`): 11/11 incl. seqlock tear rejection.

### Adapter + per-stream-process nodes (added 2026-06-26)
- `src/shm_bridge_python/shm_bridge_python/adapter.py` — **single `adapt(msg, type,
  topic)` entry**. Per-type FLAT extractors for all 25 heavy types
  (`HEAVY_FLAT.txt`): zero-copy `np.frombuffer` views, NO intermediate copy.
  CDR fallback for the other 305. Returns a `structural_key` so the recipe is
  written only when structure changes (NOT on data_size change — that rides the
  binary header every frame).
- `.../ros2_to_shm.py` — **one topic per OS process** (lowest latency: single
  thread, no executor/GIL contention). **Auto-resolves the type** from the graph
  (`get_topic_names_and_types`, like `ros2 topic type`); only `topic` is required.
  Stream name auto-derived `/a/b -> a__b`; the verbatim topic is stored in the
  recipe so the reader never reverse-engineers it.
- `.../ros2_to_shm_all.py` — **`--all` supervisor**: dynamically watches the graph
  and **spawns one `ros2_to_shm` child process per topic** (true isolation),
  reaping ones that vanish.
- `src/shm_bridge_cpp/src/ros2_to_shm.cpp` — C++ mirror: one process per stream,
  auto-resolve type, universal CDR via `GenericSubscription`, same byte contract.
- `src/shm_bridge_cpp/include/shm_bridge_cpp/flat_adapter.hpp` — typed zero-copy
  FLAT extractors for the 25 heavy types (mirror of adapter.py); returns a pointer
  into the message buffer, no copy.
- `src/shm_bridge_cpp/src/ros2_to_shm_flat.cpp` — C++ writer that wires a TYPED
  subscription + FLAT extractor for heavy types, and falls back to
  `GenericSubscription`/CDR for everything else. Links std_msgs/nav_msgs/stereo_msgs/
  map_msgs/octomap_msgs/visualization_msgs/rmw_dds_common/sensor_msgs.
  NOTE: UInt64/Int64 MultiArray go CDR (the 8-entry dtype table has no 64-bit int id).
  Verified: C++ FLAT Image -> Python reader ndarray (2,3,3) uint8; LaserScan ->
  ndarray float32; both cross-language.

#### type_name truncation (fixed)
The 64-byte header's `type_name` is `char[24]` and truncates long names
(`sensor_msgs/msg/CompressedImage` = 31 chars). The **recipe JSON carries the full
name** and the reader uses THAT for CDR reconstruction (header field is a hint only).

#### Verified (per-stream-process flow)
- `ros2_to_shm` with only `-p topic:=/x` auto-resolves type, auto-names stream.
- FLAT `Image` `/camera/rgb` and CDR `JointState`/`Imu` round-trip; recipe holds
  verbatim topic + full type.
- `--all` discovered live topics and spawned one isolated process each.
- C++ `ros2_to_shm` (auto-resolve) -> Python reader reconstructed `Imu`.

### Run examples
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# universal CDR transport (any type), Python writer:
ros2 run shm_bridge_python generic_shm_writer \
  --ros-args -p "streams:=[tf:/tf:tf2_msgs/msg/TFMessage]"

# FLAT image fast path:
ros2 run shm_bridge_python generic_shm_writer \
  --ros-args -p "streams:=[rgb:/rgb:sensor_msgs/msg/Image]"

# type-agnostic reader (python or cpp), reconstruct-only:
ros2 run shm_bridge_python generic_shm_reader --ros-args -p "streams:=[tf]"
ros2 run shm_bridge_cpp    generic_shm_reader tf

# C++ universal writer (GenericSubscription, any type):
ros2 run shm_bridge_cpp generic_shm_writer \
  --ros-args -p "streams:=[tf:/tf:tf2_msgs/msg/TFMessage]"

# --- per-stream-process nodes (lowest latency; type auto-resolved) ---

# one topic per process — only the topic is required:
ros2 run shm_bridge_python ros2_to_shm --ros-args -p topic:=/camera/color/image_raw
ros2 run shm_bridge_cpp    ros2_to_shm --ros-args -p topic:=/imu

# bridge EVERY topic, one isolated process each (dynamic):
ros2 run shm_bridge_python ros2_to_shm_all

# read back (stream name = topic with / -> __):
ros2 run shm_bridge_python generic_shm_reader \
  --ros-args -p "streams:=[camera__color__image_raw]"
```
```

# C++ API reference — `shm_bridge::Writer` / `shm_bridge::Reader`

Header: [`<shm_bridge_cpp/shm_bridge.hpp>`](../src/shm_bridge_cpp/include/shm_bridge_cpp/shm_bridge.hpp)
Link:   `-lshm_bridge_cpp -lpthread`

Everything is in namespace `shm_bridge`. No ROS is required to use the library —
ROS only appears in the example nodes that bridge to/from topics.

---

## Types

```cpp
enum class DType : uint32_t { U8, I8, U16, I16, U32, I32, F32, F64 };
enum class Encoding : uint32_t { FLAT = 0, CDR = 1 };

struct Frame {
    std::vector<uint8_t> data;     // payload bytes (FLAT buffer or CDR bytes)
    Encoding encoding = Encoding::CDR;
    uint32_t width = 0, height = 0, channels = 0;
    DType    dtype = DType::U8;
    uint64_t timestamp_ns = 0;
    uint32_t seq = 0;              // even, monotonically increasing per publish
    std::string type_name;         // e.g. "sensor_msgs/msg/Image"
};
```

---

## Writer

```cpp
shm_bridge::Writer w(const std::string& name, size_t max_bytes);
```
Creates/owns the stream `/dev/shm/<name>_{header,frame,recipe.json}`. `max_bytes`
sizes the frame buffer — make it ≥ the largest payload you will ever publish.

### Publish a FLAT (numeric) frame — the fast path
```cpp
bool w.write_flat(const void* ptr, size_t len,
                  uint32_t width, uint32_t height, uint32_t channels,
                  DType dtype, const std::string& type_name,
                  const std::string& topic = "");
```
Copies `[ptr, ptr+len)` once into shared memory, fills the shape fields, bumps the
seqlock, and calls `notify()`. Returns `false` if `len > capacity()`.

```cpp
// e.g. a 640x480 RGB image:
w.write_flat(img.data(), img.size(), 640, 480, 3,
             shm_bridge::DType::U8, "sensor_msgs/msg/Image", "/cam");
```

### Publish CDR (serialized) bytes — the type-agnostic path
```cpp
bool w.write_cdr(const void* ptr, size_t len,
                 const std::string& type_name, const std::string& topic = "");
```
Use when you already have DDS-serialized bytes for an arbitrary message type.

### Other
```cpp
void   w.notify();      // one FUTEX_WAKE for all readers (auto-called by write_*)
size_t w.capacity();    // the frame buffer size in bytes
```

---

## Reader

```cpp
shm_bridge::Reader r(const std::string& name);   // throws if the stream is absent
```

### Non-blocking snapshot of the latest frame
```cpp
bool r.read(Frame& out);   // true on a consistent (untorn) read; false otherwise
```
Implements the seqlock read/re-read/retry. Returns the latest published frame; if
you call it faster than the writer you'll see the same `seq` repeatedly.

### Blocking wait (the ~0% CPU path — use this for consumers)
```cpp
bool r.wait_and_read(Frame& out, uint64_t timeout_ns = 0);
```
`FUTEX_WAIT`s until a frame **newer** than the last one this Reader returned is
available, then reads it. `timeout_ns = 0` waits forever; otherwise returns `false`
on timeout (handy so you can check a shutdown flag periodically):
```cpp
shm_bridge::Frame f;
while (running) {
    if (r.wait_and_read(f, 100ull*1000*1000)) {   // 100 ms timeout
        // use f.data, f.width, f.height, f.seq, ...
    }
}
```

### Drop detection
```cpp
uint32_t r.last_seq();   // seq of the last frame returned (0 if none)
```
Gaps in `seq` across consecutive frames indicate the reader fell behind and missed
frames (the bridge keeps only the latest frame, by design).

---

## Lifetime & threading notes
- **One writer per stream.** The seqlock assumes a single writer. Multiple readers
  are fine and the intended use.
- `Writer`/`Reader` are not internally synchronized for concurrent use of the
  *same object* from multiple threads — give each thread its own `Reader`.
- The `Writer` creates the shm files; `Reader` only maps them read-only and throws
  if they don't exist yet (retry until the writer is up — see `shm_to_ros2.cpp`).
- Files persist in `/dev/shm` until removed/rebooted. To clean up a stream:
  `rm /dev/shm/<name>_*`.

See runnable code in [`../examples/cpp/`](../examples/cpp/):
`write_cpp.cpp`, `read_cpp.cpp`, `ros2_to_shm.cpp`, `shm_to_ros2.cpp`,
`shm_to_network.cpp`.

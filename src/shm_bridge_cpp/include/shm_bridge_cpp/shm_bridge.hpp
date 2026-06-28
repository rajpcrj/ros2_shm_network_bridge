// shm_bridge.hpp — public library API for the SHM bridge (compiled into
// libshm_bridge_cpp.so). Include this and link `shm_bridge_cpp` to read/write
// frames over the shared-memory contract from your own node — no ROS required
// for the SHM I/O itself.
//
//   #include <shm_bridge_cpp/shm_bridge.hpp>
//   shm_bridge::Writer w("rgb", 1920*1080*4);
//   w.write_flat(ptr, len, w_, h_, ch_, shm_bridge::DType::U8, "sensor_msgs/msg/Image");
//
//   shm_bridge::Reader r("rgb");
//   shm_bridge::Frame f;
//   if (r.read(f)) { /* f.data, f.width, f.height, f.encoding ... */ }
//
// Portability: source is arch-independent; the built .so is per-arch (rebuild on
// each target). The on-disk layout is little-endian (fine for x86_64 + arm64).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace shm_bridge {

enum class DType : uint32_t {
    U8 = 0, I8 = 1, U16 = 2, I16 = 3, U32 = 4, I32 = 5, F32 = 6, F64 = 7
};
enum class Encoding : uint32_t { FLAT = 0, CDR = 1 };

// A consistent frame returned by Reader::read().
struct Frame {
    std::vector<uint8_t> data;   // payload bytes (FLAT buffer or CDR bytes)
    Encoding encoding = Encoding::CDR;
    uint32_t width = 0, height = 0, channels = 0;
    DType dtype = DType::U8;
    uint64_t timestamp_ns = 0;
    uint32_t seq = 0;
    std::string type_name;       // full name (from recipe if available)
};

// Writer: owns one stream's shared memory and publishes frames under the seqlock.
class Writer {
public:
    // name -> /dev/shm/<name>_{header,frame,recipe.json}; max_bytes sizes the buffer.
    Writer(const std::string& name, size_t max_bytes);
    ~Writer();

    // Zero-copy-friendly FLAT publish: copies [ptr,len) once into shared memory.
    bool write_flat(const void* ptr, size_t len, uint32_t width, uint32_t height,
                    uint32_t channels, DType dtype, const std::string& type_name,
                    const std::string& topic = "");

    // CDR publish: copies serialized bytes once.
    bool write_cdr(const void* ptr, size_t len, const std::string& type_name,
                   const std::string& topic = "");

    // Wake all readers blocked in Reader::wait() — one O(1) FUTEX_WAKE regardless
    // of subscriber count. write_flat()/write_cdr() call this automatically after
    // a successful publish; exposed for callers that build a header by hand.
    void notify();

    size_t capacity() const;

private:
    struct Impl;
    Impl* p_;
};

// Reader: attaches to an existing stream and reconstructs consistent frames.
class Reader {
public:
    explicit Reader(const std::string& name);   // throws if shm not present
    ~Reader();

    // Returns true and fills `out` on a consistent (untorn) read; false otherwise.
    // Implements the seqlock read/re-read/retry protocol.
    bool read(Frame& out);

    // Block (≈0% CPU) until a frame newer than the last one this Reader returned
    // is available, then read it. Returns false on timeout (timeout_ns=0 waits
    // forever). This replaces busy-polling: the writer's notify() does one
    // FUTEX_WAKE that releases all waiting readers at once. Falls back to a short
    // poll if the kernel wakes spuriously.
    bool wait_and_read(Frame& out, uint64_t timeout_ns = 0);

    // The seq of the last frame returned (0 if none yet) — exposed for callers
    // that want to detect dropped frames (gaps in seq).
    uint32_t last_seq() const;

private:
    struct Impl;
    Impl* p_;
};

}  // namespace shm_bridge

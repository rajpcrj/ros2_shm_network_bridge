// shm_contract.hpp — byte-exact on-disk contract (PROMPT.md §4).
// Mirrors src/shm_bridge_python/shm_bridge_python/shm_contract.py exactly.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace shm_contract {

constexpr size_t HEADER_SIZE = 64;
constexpr uint32_t ENC_FLAT = 0;
constexpr uint32_t ENC_CDR = 1;

// dtype_id table (matches DTYPE_BY_ID in Python)
// 0=uint8 1=int8 2=uint16 3=int16 4=uint32 5=int32 6=float32 7=float64

// Fixed binary header. #pragma pack so offsets match the Python struct exactly:
//   8x uint32 (seq..reserved) then uint64 timestamp then char[24] type_name.
#pragma pack(push, 1)
struct Header {
    uint32_t seq;          // offset 0  — seqlock (even=stable, odd=writing)
    uint32_t encoding_id;  // offset 4
    uint32_t data_size;    // offset 8
    uint32_t width;        // offset 12
    uint32_t height;       // offset 16
    uint32_t channels;     // offset 20
    uint32_t dtype_id;     // offset 24
    uint32_t reserved;     // offset 28
    uint64_t timestamp_ns; // offset 32
    char type_name[24];    // offset 40 .. 63 (null-padded)
};
#pragma pack(pop)
static_assert(sizeof(Header) == HEADER_SIZE, "Header must be 64 bytes");

inline std::string header_path(const std::string& name) { return "/dev/shm/" + name + "_header"; }
inline std::string frame_path(const std::string& name)  { return "/dev/shm/" + name + "_frame"; }
inline std::string recipe_path(const std::string& name) { return "/dev/shm/" + name + "_recipe.json"; }

// Map a shm file. create=true grows to at least `size` and maps RW; otherwise RO
// using the file's existing size. Returns mapped ptr (or MAP_FAILED) and sets out_size.
inline void* map_file(const std::string& path, size_t size, bool create, size_t& out_size) {
    int flags = O_RDWR | (create ? O_CREAT : 0);
    int fd = open(path.c_str(), flags, 0666);
    if (fd < 0) { out_size = 0; return MAP_FAILED; }

    struct stat st {};
    fstat(fd, &st);
    size_t cur = static_cast<size_t>(st.st_size);
    if (create && cur < size) {
        if (ftruncate(fd, static_cast<off_t>(size)) != 0) { close(fd); out_size = 0; return MAP_FAILED; }
        cur = size;
    }
    out_size = create ? (cur < size ? size : cur) : cur;

    int prot = create ? (PROT_READ | PROT_WRITE) : PROT_READ;
    void* mem = mmap(nullptr, out_size, prot, MAP_SHARED, fd, 0);
    close(fd);
    return mem;
}

// Atomic load of the seqlock counter at offset 0 (acquire).
inline uint32_t load_seq_acquire(const void* hdr_mem) {
    const auto* a = reinterpret_cast<const std::atomic<uint32_t>*>(hdr_mem);
    return a->load(std::memory_order_acquire);
}

// Atomic store of the seqlock counter (release).
inline void store_seq_release(void* hdr_mem, uint32_t v) {
    auto* a = reinterpret_cast<std::atomic<uint32_t>*>(hdr_mem);
    a->store(v, std::memory_order_release);
}

inline void set_type_name(Header& h, const std::string& tn) {
    std::memset(h.type_name, 0, sizeof(h.type_name));
    std::memcpy(h.type_name, tn.data(), std::min(tn.size(), sizeof(h.type_name)));
}

}  // namespace shm_contract

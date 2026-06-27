// generic_shm_reader.cpp — type-agnostic shared-memory reader (PROMPT.md §6).
//
// Knows no message type. Implements the seqlock read/re-read/retry protocol and
// reports the reconstructed payload. For FLAT it exposes the raw numeric buffer
// + shape; for CDR it hands back the serialized bytes (a C++ consumer would feed
// these to rclcpp::SerializedMessage / a GenericPublisher). This proves the
// cross-language contract: it reconstructs frames written by the Python writer.
//
// Usage: generic_shm_reader <stream_name> [<stream_name> ...]
#include "shm_bridge_cpp/shm_contract.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

using namespace shm_contract;
constexpr int MAX_RETRY = 8;

struct Reader {
    std::string name;
    void* hdr = nullptr;
    void* frame = nullptr;
    size_t frame_cap = 0;

    bool open(const std::string& nm) {
        name = nm;
        size_t hs = 0, fs = 0;
        hdr = map_file(header_path(nm), HEADER_SIZE, false, hs);
        frame = map_file(frame_path(nm), 0, false, fs);
        frame_cap = fs;
        return hdr != MAP_FAILED && frame != MAP_FAILED && hs >= HEADER_SIZE;
    }

    // Returns true and fills out_hdr / out_buf on a consistent read.
    bool read_frame(Header& out_hdr, std::vector<uint8_t>& out_buf) {
        for (int i = 0; i < MAX_RETRY; ++i) {
            uint32_t s1 = load_seq_acquire(hdr);
            if (s1 & 1u) continue;                  // odd => writer mid-write
            Header h;
            std::memcpy(&h, hdr, HEADER_SIZE);
            size_t n = h.data_size;
            if (n == 0 || n > frame_cap) return false;
            out_buf.resize(n);
            std::memcpy(out_buf.data(), frame, n);  // "read the whole file"
            std::atomic_thread_fence(std::memory_order_acquire);
            uint32_t s2 = load_seq_acquire(hdr);
            if (s1 != s2) continue;                 // torn => retry
            out_hdr = h;
            return true;
        }
        return false;
    }
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <stream_name> [<stream_name> ...]\n", argv[0]);
        return 1;
    }
    std::vector<Reader> readers(argc - 1);
    for (int i = 1; i < argc; ++i) {
        if (!readers[i - 1].open(argv[i])) {
            std::fprintf(stderr, "[%s] failed to attach (start the writer first)\n", argv[i]);
        } else {
            std::printf("[%s] attached\n", argv[i]);
        }
    }

    Header h;
    std::vector<uint8_t> buf;
    uint32_t last_seq = 0;
    while (true) {
        for (auto& r : readers) {
            if (!r.hdr || r.hdr == MAP_FAILED) continue;
            if (r.read_frame(h, buf) && h.seq != last_seq) {
                last_seq = h.seq;
                if (h.encoding_id == ENC_FLAT) {
                    std::printf("[%s] seq=%u FLAT %ux%ux%u dtype_id=%u %uB (%s)\n",
                                r.name.c_str(), h.seq, h.width, h.height,
                                h.channels, h.dtype_id, h.data_size, h.type_name);
                } else {
                    std::printf("[%s] seq=%u CDR %uB (%s)\n",
                                r.name.c_str(), h.seq, h.data_size, h.type_name);
                }
                std::fflush(stdout);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}

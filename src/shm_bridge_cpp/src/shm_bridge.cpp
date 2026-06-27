// shm_bridge.cpp — implementation of the public library API (libshm_bridge_cpp.so).
#include "shm_bridge_cpp/shm_bridge.hpp"
#include "shm_bridge_cpp/shm_contract.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace shm_contract;

namespace {
uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
const char* DT_NAME[] = {"uint8","int8","uint16","int16","uint32","int32","float32","float64"};

void save_recipe(const std::string& path, const std::string& body) {
    std::string tmp = path + ".tmp";
    { std::ofstream f(tmp); f << body; }
    std::rename(tmp.c_str(), path.c_str());
}

std::string read_recipe_type(const std::string& path) {
    std::ifstream f(path);
    if (!f) return "";
    std::stringstream ss; ss << f.rdbuf();
    std::string s = ss.str();
    auto k = s.find("\"type_name\"");
    if (k == std::string::npos) return "";
    auto c = s.find(':', k);
    auto q1 = s.find('"', c + 1);
    auto q2 = s.find('"', q1 + 1);
    if (q1 == std::string::npos || q2 == std::string::npos) return "";
    return s.substr(q1 + 1, q2 - q1 - 1);
}
}  // namespace

namespace shm_bridge {

// ----------------------------- Writer -----------------------------
// Impl is defined as a struct in the public header's forward declaration; here we
// give it a full definition. Keep the seqlock write helper in this namespace.
struct Writer::Impl {
    std::string name, recipe_path;
    void* hdr = nullptr;
    void* frame = nullptr;
    size_t cap = 0;
    uint32_t seq = 0;
    std::string last_key;       // structural key -> recipe written only on change
};

Writer::Writer(const std::string& name, size_t max_bytes) : p_(new Impl) {
    p_->name = name;
    p_->recipe_path = recipe_path(name);
    size_t hs = 0, fs = 0;
    p_->hdr = map_file(header_path(name), HEADER_SIZE, true, hs);
    p_->frame = map_file(frame_path(name), max_bytes, true, fs);
    if (p_->hdr == MAP_FAILED || p_->frame == MAP_FAILED)
        throw std::runtime_error("shm_bridge::Writer: mmap failed for " + name);
    p_->cap = fs;
    Header h{};
    std::memcpy(p_->hdr, &h, HEADER_SIZE);
}

Writer::~Writer() { delete p_; }
size_t Writer::capacity() const { return p_->cap; }

namespace {
// seqlock publish: bump odd, copy payload + header, bump even.
bool seqlock_write(void* hdr, void* frame, size_t cap, uint32_t& seq,
                   const void* ptr, size_t len, Header h) {
    if (len > cap) return false;
    seq += 1;                                      // odd: writing
    store_seq_release(hdr, seq);
    std::memcpy(frame, ptr, len);
    h.seq = seq;
    h.data_size = static_cast<uint32_t>(len);
    h.timestamp_ns = now_ns();
    std::memcpy(hdr, &h, HEADER_SIZE);
    seq += 1;                                      // even: stable
    store_seq_release(hdr, seq);
    return true;
}
}  // namespace

bool Writer::write_flat(const void* ptr, size_t len, uint32_t width, uint32_t height,
                        uint32_t channels, DType dtype, const std::string& type_name,
                        const std::string& topic) {
    Header h{};
    h.encoding_id = ENC_FLAT;
    h.width = width; h.height = height; h.channels = channels;
    h.dtype_id = static_cast<uint32_t>(dtype);
    set_type_name(h, type_name);

    std::string key = "flat|" + type_name + "|" + std::to_string(h.dtype_id) + "|"
        + std::to_string(width) + "|" + std::to_string(height) + "|"
        + std::to_string(channels);
    if (key != p_->last_key) {
        std::ostringstream o;
        o << "{\"topic\":\"" << topic << "\",\"type_name\":\"" << type_name
          << "\",\"encoding\":\"flat\",\"dtype\":\"" << DT_NAME[h.dtype_id]
          << "\",\"shape\":[";
        if (channels > 1) o << height << "," << width << "," << channels;
        else if (height > 1) o << height << "," << width;
        else o << width;
        o << "]}";
        save_recipe(p_->recipe_path, o.str());
        p_->last_key = key;
    }
    return seqlock_write(p_->hdr, p_->frame, p_->cap, p_->seq, ptr, len, h);
}

bool Writer::write_cdr(const void* ptr, size_t len, const std::string& type_name,
                       const std::string& topic) {
    Header h{};
    h.encoding_id = ENC_CDR;
    set_type_name(h, type_name);
    std::string key = "cdr|" + type_name;
    if (key != p_->last_key) {
        std::ostringstream o;
        o << "{\"topic\":\"" << topic << "\",\"type_name\":\"" << type_name
          << "\",\"encoding\":\"cdr\"}";
        save_recipe(p_->recipe_path, o.str());
        p_->last_key = key;
    }
    return seqlock_write(p_->hdr, p_->frame, p_->cap, p_->seq, ptr, len, h);
}

// ----------------------------- Reader -----------------------------
struct Reader::Impl {
    std::string recipe_path;
    void* hdr = nullptr;
    void* frame = nullptr;
    size_t cap = 0;
};

Reader::Reader(const std::string& name) : p_(new Impl) {
    p_->recipe_path = recipe_path(name);
    size_t hs = 0, fs = 0;
    p_->hdr = map_file(header_path(name), HEADER_SIZE, false, hs);
    p_->frame = map_file(frame_path(name), 0, false, fs);
    if (p_->hdr == MAP_FAILED || p_->frame == MAP_FAILED || hs < HEADER_SIZE)
        throw std::runtime_error("shm_bridge::Reader: cannot attach to " + name);
    p_->cap = fs;
}

Reader::~Reader() { delete p_; }

bool Reader::read(Frame& out) {
    for (int i = 0; i < 8; ++i) {
        uint32_t s1 = load_seq_acquire(p_->hdr);
        if (s1 & 1u) continue;                     // writer mid-write
        Header h;
        std::memcpy(&h, p_->hdr, HEADER_SIZE);
        size_t n = h.data_size;
        if (n == 0 || n > p_->cap) return false;
        out.data.resize(n);
        std::memcpy(out.data.data(), p_->frame, n);
        std::atomic_thread_fence(std::memory_order_acquire);
        uint32_t s2 = load_seq_acquire(p_->hdr);
        if (s1 != s2) continue;                    // torn -> retry

        out.encoding = static_cast<Encoding>(h.encoding_id);
        out.width = h.width; out.height = h.height; out.channels = h.channels;
        out.dtype = static_cast<DType>(h.dtype_id);
        out.timestamp_ns = h.timestamp_ns;
        out.seq = h.seq;
        // full type name from recipe (header field is truncated to 24 bytes)
        std::string full = read_recipe_type(p_->recipe_path);
        out.type_name = full.empty()
            ? std::string(h.type_name, strnlen(h.type_name, sizeof(h.type_name)))
            : full;
        return true;
    }
    return false;
}

}  // namespace shm_bridge

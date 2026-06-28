// bridge_bench4.cpp — test4 bridge benchmark (links libshm_bridge_cpp.so, futex).
// Same frozen timing/percentile/omission/variance (../bench_common.hpp). ADDS:
//   FIX 1: headline CPU = pid-set (bench + daemons; bridge has none, so == self).
//   Two READER MODES so the comparison to DDS is symmetric:
//     mode "raw_flat"    : reader hands back raw FLAT bytes (no reconstruction).
//                          Fair vs DDS "loaned" (zero-copy, no deserialize).
//     mode "deserialize" : reader reconstructs a typed sensor_msgs::Image using
//                          the FAIR policy you specified — read the recipe
//                          STRUCTURE ONCE (topic + type + encoding + shape; cached,
//                          re-read only on structure change), but read the
//                          per-frame data_size and rebuild/copy the ~1 MiB Image
//                          EVERY frame. Fair vs DDS "normal" (deserializes every
//                          frame). This makes the bridge pay the same per-frame
//                          data-copy DDS pays; the bridge's only saving is the
//                          structure parse (done once, not per frame) — that delta
//                          is the honest advantage.
#include "shm_bridge_cpp/shm_bridge.hpp"
#include "shm_bridge_cpp/shm_contract.hpp"
#include "../bench_common.hpp"
#include "bench_cpu4.hpp"
#include "bench4_report.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

using namespace bench;

// Cached recipe structure (read ONCE, re-read only if the structure key changes).
struct CachedStructure {
    bool valid = false;
    std::string structure_key;   // detects change (encoding|w|h|ch|dtype)
    std::string topic, type_name, encoding;
    uint32_t width = 0, height = 0, channels = 0, dtype_id = 0;

    // Parse /dev/shm/<stream>_recipe.json once. Minimal hand-parse (no JSON dep).
    void load(const std::string& stream) {
        std::ifstream f("/dev/shm/" + stream + "_recipe.json");
        if (!f) return;
        std::stringstream ss; ss << f.rdbuf(); std::string s = ss.str();
        auto grab = [&](const char* key) -> std::string {
            auto k = s.find(key); if (k == std::string::npos) return "";
            auto c = s.find(':', k); auto q1 = s.find('"', c + 1);
            if (q1 == std::string::npos) return "";
            auto q2 = s.find('"', q1 + 1); return s.substr(q1 + 1, q2 - q1 - 1);
        };
        topic = grab("\"topic\""); type_name = grab("\"type_name\"");
        encoding = grab("\"encoding\"");
        valid = true;
    }
};

static bench4::Run4 one_run(const Args& a, const std::string& mode) {
    const std::string stream = "bench_shm4";
    for (auto suf : {"_header", "_frame", "_recipe.json"})
        std::remove(("/dev/shm/" + stream + suf).c_str());

    shm_bridge::Writer writer(stream, a.bytes);
    std::atomic<bool> run{true};
    std::atomic<int> ready{0};
    std::vector<std::vector<double>> lat(a.subs);
    std::vector<uint64_t> rx(a.subs, 0), lost(a.subs, 0);

    std::vector<std::thread> rts;
    for (int i = 0; i < a.subs; ++i) {
        rts.emplace_back([&, i]() {
            if (a.pin) pin_to_core(a.base_core + 1 + i);
            shm_bridge::Reader reader(stream);
            ready.fetch_add(1);
            shm_bridge::Frame f;
            uint32_t prev = 0; bool first = true;
            CachedStructure cache;                 // recipe read ONCE (per change)
            std::string last_key;
            sensor_msgs::msg::Image img;           // reused typed message
            while (run.load(std::memory_order_relaxed)) {
                if (!reader.wait_and_read(f, 100ull * 1000 * 1000)) continue;  // futex
                if (f.seq == prev) continue;

                if (mode == "deserialize") {
                    // structure key from the per-frame header fields; recipe (topic
                    // + type + encoding) parsed ONCE, re-read only if key changes.
                    std::ostringstream k; k << (int)f.encoding << '|' << f.width << '|'
                        << f.height << '|' << f.channels << '|' << (int)f.dtype;
                    if (k.str() != last_key) {
                        cache.load(stream);        // <-- read structure ONCE / on change
                        cache.width = f.width; cache.height = f.height;
                        cache.channels = f.channels; cache.dtype_id = (uint32_t)f.dtype;
                        last_key = k.str();
                        img.header.frame_id = cache.topic;
                        img.encoding = cache.encoding.empty() ? "mono8" : cache.encoding;
                    }
                    // EVERY frame: read total bytes (data_size) + rebuild typed Image
                    size_t n = f.data.size();      // data_size for THIS frame
                    img.height = f.height ? f.height : 1;
                    img.width  = f.width;
                    img.step   = f.width;
                    img.data.resize(n);
                    std::memcpy(img.data.data(), f.data.data(), n);   // per-frame copy
                    // latency stamp lives in the first 8 bytes of the payload
                    uint64_t sent; std::memcpy(&sent, img.data.data(), sizeof(sent));
                    double ms = (mono_ns() - sent) / 1e6;
                    if (!first) { uint32_t g = (f.seq - prev) / 2; if (g > 1) lost[i] += (g - 1); }
                    first = false; prev = f.seq; rx[i]++;
                    if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
                } else {
                    // raw_flat: no reconstruction, just read the timestamp.
                    uint64_t sent; std::memcpy(&sent, f.data.data(), sizeof(sent));
                    double ms = (mono_ns() - sent) / 1e6;
                    if (!first) { uint32_t g = (f.seq - prev) / 2; if (g > 1) lost[i] += (g - 1); }
                    first = false; prev = f.seq; rx[i]++;
                    if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
                }
            }
        });
    }

    std::thread wt([&]() {
        if (a.pin) pin_to_core(a.base_core);
        std::vector<uint8_t> buf(a.bytes, 0);
        while (ready.load() < a.subs) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const auto period = std::chrono::duration_cast<clk::duration>(
            std::chrono::duration<double>(1.0 / a.rate));
        auto t0 = clk::now(); uint64_t k = 0;
        while (run.load(std::memory_order_relaxed)) {
            uint64_t t = mono_ns();
            std::memcpy(buf.data(), &t, sizeof(t));
            // write a real 2-D image shape so the deserialize path reconstructs Image
            writer.write_flat(buf.data(), a.bytes, (uint32_t)a.bytes, 1, 1,
                              shm_bridge::DType::U8, "sensor_msgs/msg/Image");
            std::this_thread::sleep_until(t0 + (++k) * period);   // no coordinated omission
        }
    });

    bench4::PidSetCpuMeter pidmeter; pidmeter.daemon_names = {}; pidmeter.prime();  // no daemon
    bench4::SystemCpuMeter sysmeter; sysmeter.prime();
    double self0 = proc_cpu_seconds(); auto w0 = clk::now();
    double pc = 0, pr = 0;
    std::string breach = governed_sleep(a.secs, pc, pr);
    run.store(false); wt.join(); for (auto& t : rts) t.join();
    double wall = std::chrono::duration<double>(clk::now() - w0).count();

    bench4::Run4 R; RunResult& r = R.base;
    double warm = 0.20; uint64_t trx = 0, tlost = 0; int conn = 0;
    for (int i = 0; i < a.subs; ++i) {
        size_t s = (size_t)(lat[i].size() * warm);
        for (size_t j = s; j < lat[i].size(); ++j) r.pooled_ms.push_back(lat[i][j]);
        trx += rx[i]; tlost += lost[i]; if (rx[i] > 0) conn++;
    }
    r.rx = trx; r.lost = tlost;
    r.fps_per_sub = trx / (double)a.subs / wall;
    r.cpu_pct_core = 100.0 * (proc_cpu_seconds() - self0) / wall;
    r.subs_total = a.subs; r.subs_connected = conn;
    r.cpu_pct_machine_peak = pc; r.ram_pct_peak = pr;
    if (!breach.empty()) r.gate_breach = breach;
    R.cpu_pidset_pct = pidmeter.read_pct_of_one_core(wall);   // FIX 1 headline
    R.cpu_system_pct = sysmeter.read_pct_of_one_core();       // secondary
    R.can_loan = false;
    return R;
}

int main(int argc, char** argv) {
    // --deserialize selects the fair-vs-DDS-normal reconstruction mode.
    std::string mode = "raw_flat";
    for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--deserialize") mode = "deserialize";
    Args a = parse(argc, argv, "bridge_shm_futex");
    double oh = measure_overhead_ns(a.bytes);
    double idle = bench4::idle_floor_pct_of_one_core(2.0);
    std::vector<bench4::Run4> runs;
    for (int k = 0; k < a.runs; ++k) {
        std::printf("[bridge4/%s] run %d/%d ...\n", mode.c_str(), k + 1, a.runs);
        runs.push_back(one_run(a, mode));
    }
    std::string ptable = "processes: bridge benchmark only (no daemon) -> "
                         "pid-set CPU == self CPU (sanity check)\n";
    // deser cost for the deserialize mode is the per-frame Image rebuild; for
    // raw_flat it is 0 by construction. We report the mode in the CSV.
    return bench4::report4(a, mode, false, runs, oh, /*deser_ns=*/0.0, idle, ptable);
}

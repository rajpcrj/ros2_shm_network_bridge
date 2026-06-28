// shm_perf_test.cpp — Apex.AI performance_test-style benchmark for the SHM bridge,
// built ON TOP of libshm_bridge_cpp.so (links the library; no ROS, no DDS).
//
// Model (matches performance_test's single-process design):
//   ONE process. 1 writer thread publishes a fixed payload at a fixed rate into
//   /dev/shm; N reader threads each block in Reader::wait_and_read() (FUTEX — no
//   busy-poll) and timestamp every frame. End-to-end latency is measured with a
//   monotonic timestamp the writer embeds in the FLAT payload's first 8 bytes, so
//   it is independent of the bridge header clock and identical on both sides
//   (same process, same steady_clock).
//
// Why this is the fair apples-to-apples vs DDS performance_test:
//   - single process, threads not processes  -> no per-subscriber process tax
//   - futex wake  -> readers cost ~0% CPU while idle; writer wake is O(1) for all N
//   - one shared frame buffer  -> RAM is O(1) in subscriber count
//
// Reports (performance_test-style): per-subscriber and pooled latency
// mean/min/max/p50/p90/p99/p999, delivered samples, LOST samples (seq gaps),
// and process CPU%. CSV line for scripting.
//
// Usage:
//   shm_perf_test --subs N --bytes B --rate HZ --secs T [--csv path] [--poll]
//     --subs   number of reader threads (default 4)
//     --bytes  payload size in bytes (default 1048320 ~= 640x546x3)
//     --rate   publish rate Hz (default 30)
//     --secs   measurement seconds (default 10)
//     --poll   use busy-poll read() instead of futex wait_and_read() (for A/B)
#include "shm_bridge_cpp/shm_bridge.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using clk = std::chrono::steady_clock;
static inline uint64_t mono_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        clk::now().time_since_epoch()).count();
}

struct Args {
    int subs = 4;
    size_t bytes = 1048320;
    double rate = 30.0;
    double secs = 10.0;
    std::string csv;
    bool poll = false;
};

static Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto next = [&]() { return (i + 1 < argc) ? argv[++i] : ""; };
        if (s == "--subs") a.subs = std::atoi(next());
        else if (s == "--bytes") a.bytes = static_cast<size_t>(std::atoll(next()));
        else if (s == "--rate") a.rate = std::atof(next());
        else if (s == "--secs") a.secs = std::atof(next());
        else if (s == "--csv") a.csv = next();
        else if (s == "--poll") a.poll = true;
    }
    return a;
}

// per-reader collected latencies + lost-sample accounting
struct SubStats {
    std::vector<double> lat_ms;
    uint64_t received = 0;
    uint64_t lost = 0;        // gaps in seq (frames the reader never saw)
};

static double pct(std::vector<double>& v, double p) {
    if (v.empty()) return NAN;
    size_t k = std::min(v.size() - 1,
                        static_cast<size_t>(p / 100.0 * v.size()));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

// read process CPU time (utime+stime) in seconds, for CPU% over the window
static double proc_cpu_seconds() {
    std::ifstream f("/proc/self/stat");
    std::string tok;
    long utime = 0, stime = 0;
    for (int i = 1; f >> tok; ++i) {
        if (i == 14) utime = std::stol(tok);
        else if (i == 15) { stime = std::stol(tok); break; }
    }
    long hz = sysconf(_SC_CLK_TCK);
    return (utime + stime) / static_cast<double>(hz);
}

int main(int argc, char** argv) {
    Args a = parse(argc, argv);
    const std::string stream = "perf_shm";
    // clean any stale stream
    std::remove(("/dev/shm/" + stream + "_header").c_str());
    std::remove(("/dev/shm/" + stream + "_frame").c_str());
    std::remove(("/dev/shm/" + stream + "_recipe.json").c_str());

    shm_bridge::Writer writer(stream, a.bytes);
    std::atomic<bool> run{true};
    std::atomic<int> ready{0};

    // ---- writer thread: publish fixed payload @ rate, stamp mono_ns in bytes[0:8]
    std::thread wt([&]() {
        std::vector<uint8_t> buf(a.bytes, 0);
        const auto period = std::chrono::duration<double>(1.0 / a.rate);
        auto next = clk::now();
        // wait until readers attached so we don't waste the first frames
        while (ready.load() < a.subs) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        while (run.load()) {
            uint64_t t = mono_ns();
            std::memcpy(buf.data(), &t, sizeof(t));   // embed send time
            writer.write_flat(buf.data(), a.bytes, static_cast<uint32_t>(a.bytes),
                              1, 1, shm_bridge::DType::U8, "perf/Bytes");
            next += std::chrono::duration_cast<clk::duration>(period);
            std::this_thread::sleep_until(next);
        }
    });

    // ---- N reader threads: block on futex (or poll), measure latency + losses
    std::vector<SubStats> stats(a.subs);
    std::vector<std::thread> rts;
    for (int i = 0; i < a.subs; ++i) {
        rts.emplace_back([&, i]() {
            shm_bridge::Reader reader(stream);
            ready.fetch_add(1);
            shm_bridge::Frame f;
            uint32_t prev_seq = 0;
            bool first = true;
            while (run.load()) {
                bool got = a.poll ? reader.read(f)
                                  : reader.wait_and_read(f, 100ull * 1000 * 1000);
                if (!got) continue;
                if (f.seq == prev_seq) continue;          // same frame, skip
                uint64_t sent;
                std::memcpy(&sent, f.data.data(), sizeof(sent));
                double lat = (mono_ns() - sent) / 1e6;
                // lost = stable frames between prev and now we missed.
                // seq advances by 2 per publish (odd then even); count even gaps.
                if (!first) {
                    uint32_t gap = (f.seq - prev_seq) / 2;
                    if (gap > 1) stats[i].lost += (gap - 1);
                }
                first = false;
                prev_seq = f.seq;
                stats[i].received++;
                if (lat >= 0 && lat < 60000) stats[i].lat_ms.push_back(lat);
            }
        });
    }

    double cpu0 = proc_cpu_seconds();
    auto wall0 = clk::now();
    std::this_thread::sleep_for(std::chrono::duration<double>(a.secs));
    run.store(false);
    wt.join();
    for (auto& t : rts) t.join();
    double cpu_s = proc_cpu_seconds() - cpu0;
    double wall_s = std::chrono::duration<double>(clk::now() - wall0).count();
    int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    double cpu_pct = 100.0 * cpu_s / wall_s;             // % of one core
    double cpu_pct_sys = cpu_pct / ncpu;                 // % of whole machine

    // pool + per-sub summary
    std::vector<double> pooled;
    uint64_t total_rx = 0, total_lost = 0;
    double warm = 0.20;
    for (auto& s : stats) {
        size_t k = static_cast<size_t>(s.lat_ms.size() * warm);
        for (size_t j = k; j < s.lat_ms.size(); ++j) pooled.push_back(s.lat_ms[j]);
        total_rx += s.received;
        total_lost += s.lost;
    }
    auto P = [&](double p) { return pct(pooled, p); };
    double mean = 0; for (double x : pooled) mean += x;
    if (!pooled.empty()) mean /= pooled.size();
    double mn = pooled.empty() ? NAN : *std::min_element(pooled.begin(), pooled.end());
    double mx = pooled.empty() ? NAN : *std::max_element(pooled.begin(), pooled.end());
    double exp_per_sub = a.rate * a.secs;
    double fps_per_sub = total_rx / static_cast<double>(a.subs) / wall_s;

    std::printf("\n=== shm_perf_test (single process, 1 writer + %d reader threads) ===\n", a.subs);
    std::printf("mode=%s  payload=%zu B  rate=%.0f Hz  secs=%.0f  cores=%d\n",
                a.poll ? "BUSY-POLL" : "FUTEX", a.bytes, a.rate, a.secs, ncpu);
    std::printf("latency ms : mean=%.3f min=%.3f p50=%.3f p90=%.3f p99=%.3f p999=%.3f max=%.3f\n",
                mean, mn, P(50), P(90), P(99), P(99.9), mx);
    std::printf("delivered  : %.1f fps/sub  rx=%llu  lost=%llu (%.3f%%)  expected/sub=%.0f\n",
                fps_per_sub, (unsigned long long)total_rx, (unsigned long long)total_lost,
                total_rx ? 100.0 * total_lost / (total_rx + total_lost) : 0.0, exp_per_sub);
    std::printf("CPU        : %.1f%% of one core  (%.1f%% of %d-core machine)\n",
                cpu_pct, cpu_pct_sys, ncpu);

    if (!a.csv.empty()) {
        bool exists = std::ifstream(a.csv).good();
        std::ofstream f(a.csv, std::ios::app);
        if (!exists)
            f << "mode,subs,bytes,rate,secs,mean_ms,min_ms,p50_ms,p90_ms,p99_ms,"
                 "p999_ms,max_ms,fps_per_sub,rx,lost,lost_pct,cpu_pct_core,cpu_pct_machine\n";
        f << (a.poll ? "poll" : "futex") << "," << a.subs << "," << a.bytes << ","
          << a.rate << "," << a.secs << "," << mean << "," << mn << "," << P(50) << ","
          << P(90) << "," << P(99) << "," << P(99.9) << "," << mx << "," << fps_per_sub
          << "," << total_rx << "," << total_lost << ","
          << (total_rx ? 100.0 * total_lost / (total_rx + total_lost) : 0.0) << ","
          << cpu_pct << "," << cpu_pct_sys << "\n";
        std::printf("appended CSV -> %s\n", a.csv.c_str());
    }
    return 0;
}

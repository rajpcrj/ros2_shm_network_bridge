// bench_common.hpp — shared, transport-agnostic benchmark scaffolding so the
// THREE benchmark binaries (bridge_bench / fastdds_bench / cyclonedds_bench)
// measure latency with byte-identical logic. Only the transport differs.
//
// Methodology (industry latency-benchmarking principles):
//  - Distributions, not averages: report p50/p90/p99/p99.9/max (lead with tail).
//  - Warm-up discarded; steady-state window.
//  - Multiple runs + variance: caller runs K times; we report mean±stdev across runs.
//  - Isolate one variable: identical timing/payload/rate here; only pub/sub swaps.
//  - Coordinated omission avoided: publisher uses an absolute fixed schedule
//    (sleep_until(t0 + k*period)); it NEVER waits on subscribers. If a reader
//    falls behind, the frame is counted as lost (seq gap), not hidden.
//  - Measurement overhead disclosed: measure_overhead_ns() times the
//    timestamp+memcpy path so it can be reported/subtracted.
//  - Environment: pin_to_core() sets CPU affinity; caller reports governor/turbo.
//
// Latency clock: a CLOCK_MONOTONIC (steady_clock) stamp embedded in the first 8
// bytes of the payload by the publisher; the subscriber subtracts on receipt.
// Both run in the same process -> same clock, no cross-host skew.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <numeric>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <unistd.h>

namespace bench {

using clk = std::chrono::steady_clock;

inline uint64_t mono_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        clk::now().time_since_epoch()).count();
}

struct Args {
    int subs = 1;
    size_t bytes = 1048320;     // 640x546x3 ~= 1.0 MiB
    double rate = 30.0;         // Hz
    double secs = 10.0;         // measurement window per run
    int runs = 5;               // repeated runs for variance
    int base_core = 0;          // pin writer to base_core, readers to base_core+i
    bool pin = true;
    std::string csv;
    std::string transport;      // filled by each binary
};

inline Args parse(int argc, char** argv, const std::string& transport) {
    Args a; a.transport = transport;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto nx = [&]() { return (i + 1 < argc) ? argv[++i] : ""; };
        if (s == "--subs") a.subs = std::atoi(nx());
        else if (s == "--bytes") a.bytes = (size_t)std::atoll(nx());
        else if (s == "--rate") a.rate = std::atof(nx());
        else if (s == "--secs") a.secs = std::atof(nx());
        else if (s == "--runs") a.runs = std::atoi(nx());
        else if (s == "--no-pin") a.pin = false;
        else if (s == "--base-core") a.base_core = std::atoi(nx());
        else if (s == "--csv") a.csv = nx();
    }
    return a;
}

inline void pin_to_core(int core) {
    if (core < 0) return;
    int n = (int)sysconf(_SC_NPROCESSORS_ONLN);
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(core % n, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

// percentile on a copy (caller already concatenated + warm-up-trimmed)
inline double pct(std::vector<double> v, double p) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    size_t k = std::min(v.size() - 1, (size_t)std::llround(p / 100.0 * (v.size() - 1)));
    return v[k];
}
inline double mean_of(const std::vector<double>& v) {
    if (v.empty()) return NAN;
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
inline double stdev_of(const std::vector<double>& v) {
    if (v.size() < 2) return 0.0;
    double m = mean_of(v), s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / (v.size() - 1));
}

// Per-run result (one row); pooled latency across all subs of that run.
struct RunResult {
    std::vector<double> pooled_ms;   // warm-up-trimmed latencies, all subs pooled
    uint64_t rx = 0, lost = 0;
    double fps_per_sub = 0;
    double cpu_pct_core = 0;         // % of one core (process utime+stime / wall)
    int subs_total = 0;             // subscribers launched this run
    int subs_connected = 0;         // subscribers that received >=1 frame
    double cpu_pct_machine_peak = 0;// peak system-wide CPU% during the run
    double ram_pct_peak = 0;        // peak system RAM% during the run
    std::string gate_breach;        // "CPU"/"RAM"/"" — set if a gate tripped
};

// System-wide CPU% (delta) + RAM% — for the safety governor. Call sample_cpu()
// once to prime, then again to read the %CPU since the prior call.
inline std::pair<uint64_t,uint64_t> cpu_raw() {
    std::ifstream f("/proc/stat"); std::string c; f >> c;
    uint64_t v, total = 0, idle = 0; int i = 0;
    while (f >> v) { total += v; if (i == 3 || i == 4) idle += v; i++; }
    return {total, idle};
}
inline double ram_pct_now() {
    long mt = 0, ma = 0; std::ifstream f("/proc/meminfo"); std::string k; long v; std::string u;
    while (f >> k >> v >> u) { if (k == "MemTotal:") mt = v; else if (k == "MemAvailable:") { ma = v; break; } }
    return mt ? 100.0 * (1 - (double)ma / mt) : 0.0;
}

inline double proc_cpu_seconds() {
    std::ifstream f("/proc/self/stat");
    std::string tok; long ut = 0, st = 0;
    for (int i = 1; f >> tok; ++i) {
        if (i == 14) ut = std::stol(tok);
        else if (i == 15) { st = std::stol(tok); break; }
    }
    return (ut + st) / (double)sysconf(_SC_CLK_TCK);
}

// Sleep for `secs` while sampling system CPU%/RAM% each second; record peaks.
// Returns "" if no gate tripped, else "CPU"/"RAM" (gate = 95%). The caller still
// runs the full window (we don't truncate latency data mid-run); the gate result
// tells the outer sweep to STOP launching higher N.
inline std::string governed_sleep(double secs, double& peak_cpu, double& peak_ram,
                                  double cpu_gate = 95.0, double ram_gate = 95.0) {
    std::string breach;
    auto prev = cpu_raw();
    int whole = (int)secs;
    for (int t = 0; t < whole; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto cur = cpu_raw();
        double dt = (double)(cur.first - prev.first), di = (double)(cur.second - prev.second);
        double cpu = dt > 0 ? 100.0 * (1 - di / dt) : 0.0;
        double ram = ram_pct_now();
        prev = cur;
        if (cpu > peak_cpu) peak_cpu = cpu;
        if (ram > peak_ram) peak_ram = ram;
        if (breach.empty() && cpu > cpu_gate) breach = "CPU";
        if (breach.empty() && ram > ram_gate) breach = "RAM";
    }
    double frac = secs - whole;
    if (frac > 0) std::this_thread::sleep_for(std::chrono::duration<double>(frac));
    return breach;
}

// Measure the pure timestamp+memcpy overhead of the hot path (disclosed, not hidden)
inline double measure_overhead_ns(size_t bytes) {
    std::vector<uint8_t> a(bytes), b(bytes);
    const int iters = 2000;
    uint64_t t0 = mono_ns();
    for (int i = 0; i < iters; ++i) {
        uint64_t t = mono_ns();
        std::memcpy(a.data(), &t, sizeof(t));
        std::memcpy(b.data(), a.data(), bytes);   // emulate one frame copy
        asm volatile("" ::"r"(b.data()) : "memory");
    }
    return (mono_ns() - t0) / (double)iters;
}

inline std::string env_note() {
    std::string gov = "unknown";
    std::ifstream g("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    if (g) std::getline(g, gov);
    std::string turbo = "?";
    std::ifstream t("/sys/devices/system/cpu/intel_pstate/no_turbo");
    if (t) { std::string v; std::getline(t, v); turbo = (v == "0") ? "on" : "off"; }
    return "governor=" + gov + " turbo=" + turbo +
           " cores=" + std::to_string(sysconf(_SC_NPROCESSORS_ONLN));
}

// Aggregate K runs: print mean±stdev of p50/p99/etc ACROSS runs, plus pooled.
// Returns an exit code: 0 ok, 2 = a subscriber lost subscription, 3 = CPU/RAM
// gate breached — so the outer sweep can STOP at this N.
inline int report(const Args& a, std::vector<RunResult>& runs, double overhead_ns) {
    auto across = [&](auto getter) {
        std::vector<double> v; for (auto& r : runs) v.push_back(getter(r));
        return std::make_pair(mean_of(v), stdev_of(v));
    };
    auto p50 = across([](RunResult& r){ return pct(r.pooled_ms, 50); });
    auto p90 = across([](RunResult& r){ return pct(r.pooled_ms, 90); });
    auto p99 = across([](RunResult& r){ return pct(r.pooled_ms, 99); });
    auto p999 = across([](RunResult& r){ return pct(r.pooled_ms, 99.9); });
    auto pmax = across([](RunResult& r){ return r.pooled_ms.empty() ? NAN :
                          *std::max_element(r.pooled_ms.begin(), r.pooled_ms.end()); });
    auto pmean = across([](RunResult& r){ return mean_of(r.pooled_ms); });
    auto fps = across([](RunResult& r){ return r.fps_per_sub; });
    auto cpu = across([](RunResult& r){ return r.cpu_pct_core; });
    double lostpct = 0; uint64_t rx = 0, lost = 0;
    for (auto& r : runs) { rx += r.rx; lost += r.lost; }
    if (rx + lost) lostpct = 100.0 * lost / (rx + lost);

    std::printf("\n=== %s  (single process: 1 writer + %d reader threads) ===\n",
                a.transport.c_str(), a.subs);
    std::printf("payload=%zu B  rate=%.0f Hz  secs=%.0f  runs=%d  %s  pin=%s\n",
                a.bytes, a.rate, a.secs, a.runs, env_note().c_str(),
                a.pin ? "yes" : "no");
    std::printf("measurement overhead ~= %.0f ns/frame (timestamp+1 memcpy)\n", overhead_ns);
    std::printf("latency ms (mean +/- stdev ACROSS %d runs):\n", a.runs);
    std::printf("  mean=%.3f+/-%.3f  p50=%.3f+/-%.3f  p90=%.3f+/-%.3f\n",
                pmean.first, pmean.second, p50.first, p50.second, p90.first, p90.second);
    std::printf("  p99=%.3f+/-%.3f  p99.9=%.3f+/-%.3f  max=%.3f+/-%.3f\n",
                p99.first, p99.second, p999.first, p999.second, pmax.first, pmax.second);
    std::printf("delivered: %.1f+/-%.1f fps/sub   lost=%.3f%% (rx=%llu lost=%llu)\n",
                fps.first, fps.second, lostpct,
                (unsigned long long)rx, (unsigned long long)lost);
    std::printf("CPU: %.1f+/-%.1f %% of one core\n", cpu.first, cpu.second);

    // ---- safety/health signals (parsed by the sweep runner) ----
    int subs_total = runs.empty() ? a.subs : runs.back().subs_total;
    int min_conn = subs_total;
    double peak_cpu = 0, peak_ram = 0; std::string breach;
    for (auto& r : runs) {
        min_conn = std::min(min_conn, r.subs_connected);
        peak_cpu = std::max(peak_cpu, r.cpu_pct_machine_peak);
        peak_ram = std::max(peak_ram, r.ram_pct_peak);
        if (breach.empty() && !r.gate_breach.empty()) breach = r.gate_breach;
    }
    bool sub_lost = min_conn < subs_total;
    std::printf("HEALTH subs_connected=%d/%d peak_cpu=%.1f%% peak_ram=%.1f%% "
                "gate=%s sub_lost=%s\n", min_conn, subs_total, peak_cpu, peak_ram,
                breach.empty() ? "none" : breach.c_str(), sub_lost ? "YES" : "no");

    if (!a.csv.empty()) {
        bool ex = std::ifstream(a.csv).good();
        std::ofstream f(a.csv, std::ios::app);
        if (!ex) f << "transport,subs,bytes,rate,secs,runs,"
                      "mean_ms,mean_sd,p50_ms,p50_sd,p90_ms,p90_sd,p99_ms,p99_sd,"
                      "p999_ms,p999_sd,max_ms,max_sd,fps_per_sub,fps_sd,"
                      "lost_pct,cpu_pct_core,cpu_sd,overhead_ns,"
                      "subs_connected,subs_total,peak_cpu,peak_ram,gate,sub_lost\n";
        f << a.transport << "," << a.subs << "," << a.bytes << "," << a.rate << ","
          << a.secs << "," << a.runs << ","
          << pmean.first << "," << pmean.second << "," << p50.first << "," << p50.second << ","
          << p90.first << "," << p90.second << "," << p99.first << "," << p99.second << ","
          << p999.first << "," << p999.second << "," << pmax.first << "," << pmax.second << ","
          << fps.first << "," << fps.second << "," << lostpct << ","
          << cpu.first << "," << cpu.second << "," << overhead_ns << ","
          << min_conn << "," << subs_total << "," << peak_cpu << "," << peak_ram << ","
          << (breach.empty() ? "none" : breach) << "," << (sub_lost ? 1 : 0) << "\n";
        std::printf("appended CSV -> %s\n", a.csv.c_str());
    }
    if (!breach.empty()) return 3;
    if (sub_lost) return 2;
    return 0;
}

}  // namespace bench

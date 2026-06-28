// bench_cpu4.hpp — FIX 1: symmetric WHOLE-SYSTEM CPU accounting for test4.
//
// Problem with the original headline CPU metric: it used /proc/self/stat (this
// process only). That is complete for the bridge (no daemon) but MISSES
// CycloneDDS's iox-roudi daemon and any FastDDS helper threads/processes —
// making DDS look unfairly cheap. This header measures CPU the SAME way for all
// transports: integrated whole-system busy time over the steady-state window,
// idle-subtracted, expressed as "% of one core" (system-busy-fraction × cores)
// so it is directly comparable to the per-core numbers already reported.
//
// It REUSES the frozen primitives from ../bench_common.hpp (cpu_raw(),
// proc_cpu_seconds(), ram_pct_now()). It does NOT modify them.
#pragma once

#include "../bench_common.hpp"   // frozen: cpu_raw(), proc_cpu_seconds(), ram_pct_now()

#include <cstdio>
#include <dirent.h>
#include <string>
#include <thread>
#include <vector>

namespace bench4 {

// Whole-system busy fraction between two cpu_raw() snapshots.
// cpu_raw() returns {total_jiffies, idle_jiffies} aggregated over all cores.
inline double system_busy_pct_of_one_core(std::pair<uint64_t,uint64_t> a,
                                          std::pair<uint64_t,uint64_t> b) {
    double dt = double(b.first - a.first);
    double di = double(b.second - a.second);
    if (dt <= 0) return 0.0;
    double busy_frac = 1.0 - di / dt;                  // fraction of ALL cores busy
    int ncpu = (int)sysconf(_SC_NPROCESSORS_ONLN);
    return 100.0 * busy_frac * ncpu;                   // "% of one core" units
}

// Sample the idle whole-system CPU floor for `secs` seconds (call before a run).
// Returns the background busy level in "% of one core". Confirms the system-CPU
// number during the run is attributable to the benchmark, not background load.
inline double idle_floor_pct_of_one_core(double secs = 2.0) {
    auto a = bench::cpu_raw();
    std::this_thread::sleep_for(std::chrono::duration<double>(secs));
    auto b = bench::cpu_raw();
    return system_busy_pct_of_one_core(a, b);
}

// A live whole-system CPU integrator: prime() then read() at window end.
struct SystemCpuMeter {
    std::pair<uint64_t,uint64_t> start{};
    void prime() { start = bench::cpu_raw(); }
    double read_pct_of_one_core() const {
        return system_busy_pct_of_one_core(start, bench::cpu_raw());
    }
};

// ---- process enumeration: list every process a transport spawned, with CPU ----
// Reads /proc to find processes whose comm matches one of `names`. Returns lines
// "pid comm utime+stime_jiffies". Used to (a) print what each transport spawned,
// (b) sanity-check iox-roudi shows non-zero CPU during the Cyclone run.
struct ProcInfo { int pid; std::string comm; long cpu_jiffies; };

inline long proc_cpu_jiffies(int pid) {
    std::string p = "/proc/" + std::to_string(pid) + "/stat";
    std::ifstream f(p);
    if (!f) return -1;
    std::string tok; long ut = 0, st = 0;
    for (int i = 1; f >> tok; ++i) {
        if (i == 14) ut = std::stol(tok);
        else if (i == 15) { st = std::stol(tok); break; }
    }
    return ut + st;
}

inline std::string proc_comm(int pid) {
    std::ifstream f("/proc/" + std::to_string(pid) + "/comm");
    std::string c; std::getline(f, c); return c;
}

// Find all PIDs whose comm contains any of the given substrings.
inline std::vector<ProcInfo> find_procs(const std::vector<std::string>& subs) {
    std::vector<ProcInfo> out;
    DIR* d = opendir("/proc");
    if (!d) return out;
    struct dirent* e;
    while ((e = readdir(d))) {
        if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
        int pid = atoi(e->d_name);
        std::string comm = proc_comm(pid);
        for (auto& s : subs) {
            if (!comm.empty() && comm.find(s) != std::string::npos) {
                out.push_back({pid, comm, proc_cpu_jiffies(pid)});
                break;
            }
        }
    }
    closedir(d);
    return out;
}

// Snapshot CPU jiffies for a set of PIDs (for before/after delta during a run).
inline void snapshot(std::vector<ProcInfo>& procs) {
    for (auto& p : procs) p.cpu_jiffies = proc_cpu_jiffies(p.pid);
}

// HEADLINE CPU metric (robust to desktop activity): sum CPU of a PID SET =
// this benchmark process + any process whose comm matches `daemon_names`
// (e.g. "iox-roudi", FastDDS helpers). Counts the daemons the original self-CPU
// metric missed, but IGNORES unrelated processes (your browser/editor), so you
// can use the machine during the run without polluting the number.
//   prime() at window start, read_pct_of_one_core() at window end.
struct PidSetCpuMeter {
    std::vector<std::string> daemon_names;
    long start_jiffies = 0;
    std::vector<int> pids;

    long sum_jiffies() {
        long total = 0;
        pids.clear();
        int self = (int)getpid();
        long s = proc_cpu_jiffies(self);
        if (s >= 0) { total += s; pids.push_back(self); }
        for (auto& pi : find_procs(daemon_names)) {
            if (pi.pid == self) continue;          // already counted
            long j = proc_cpu_jiffies(pi.pid);
            if (j >= 0) { total += j; pids.push_back(pi.pid); }
        }
        return total;
    }
    void prime() { start_jiffies = sum_jiffies(); }
    double read_pct_of_one_core(double wall_s) {
        long end = sum_jiffies();
        long hz = sysconf(_SC_CLK_TCK);
        if (wall_s <= 0) return 0.0;
        return 100.0 * ((end - start_jiffies) / (double)hz) / wall_s;
    }
};

inline void print_proc_table(const std::string& transport,
                             const std::vector<ProcInfo>& before,
                             const std::vector<ProcInfo>& after, double wall_s) {
    long hz = sysconf(_SC_CLK_TCK);
    std::printf("processes captured by whole-system CPU for [%s]:\n", transport.c_str());
    for (size_t i = 0; i < after.size(); ++i) {
        long d = (i < before.size() && before[i].pid == after[i].pid)
                 ? (after[i].cpu_jiffies - before[i].cpu_jiffies)
                 : after[i].cpu_jiffies;
        double pct = wall_s > 0 ? 100.0 * (d / (double)hz) / wall_s : 0.0;
        std::printf("  pid=%-7d %-16s  %.1f%% of one core\n",
                    after[i].pid, after[i].comm.c_str(), pct);
    }
}

}  // namespace bench4

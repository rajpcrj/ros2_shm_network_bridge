// bridge_bench.cpp — benchmark the /dev/shm bridge via libshm_bridge_cpp.so.
// 1 writer thread (fixed-rate, coordinated-omission-safe) + N reader threads,
// each blocking in Reader::wait_and_read() (FUTEX, ~0% idle CPU). Shared timing
// logic in bench_common.hpp so it is identical to the DDS binaries.
#include "shm_bridge_cpp/shm_bridge.hpp"
#include "bench_common.hpp"

#include <atomic>
#include <thread>
#include <vector>

using namespace bench;

static RunResult one_run(const Args& a) {
    const std::string stream = "bench_shm";
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
            while (run.load(std::memory_order_relaxed)) {
                if (!reader.wait_and_read(f, 100ull * 1000 * 1000)) continue;
                if (f.seq == prev) continue;
                uint64_t sent; std::memcpy(&sent, f.data.data(), sizeof(sent));
                double ms = (mono_ns() - sent) / 1e6;
                if (!first) { uint32_t g = (f.seq - prev) / 2; if (g > 1) lost[i] += (g - 1); }
                first = false; prev = f.seq; rx[i]++;
                if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
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
            writer.write_flat(buf.data(), a.bytes, (uint32_t)a.bytes, 1, 1,
                              shm_bridge::DType::U8, "perf/Bytes");
            // absolute schedule -> NO coordinated omission (never waits on readers)
            std::this_thread::sleep_until(t0 + (++k) * period);
        }
    });

    double cpu0 = proc_cpu_seconds(); auto w0 = clk::now();
    double pc = 0, pr = 0;
    std::string breach = governed_sleep(a.secs, pc, pr);
    run.store(false); wt.join(); for (auto& t : rts) t.join();
    double wall = std::chrono::duration<double>(clk::now() - w0).count();

    RunResult r; double warm = 0.20; uint64_t trx = 0, tlost = 0; int conn = 0;
    for (int i = 0; i < a.subs; ++i) {
        size_t s = (size_t)(lat[i].size() * warm);
        for (size_t j = s; j < lat[i].size(); ++j) r.pooled_ms.push_back(lat[i][j]);
        trx += rx[i]; tlost += lost[i];
        if (rx[i] > 0) conn++;
    }
    r.rx = trx; r.lost = tlost;
    r.fps_per_sub = trx / (double)a.subs / wall;
    r.cpu_pct_core = 100.0 * (proc_cpu_seconds() - cpu0) / wall;
    r.subs_total = a.subs; r.subs_connected = conn;
    r.cpu_pct_machine_peak = pc; r.ram_pct_peak = pr;
    if (!breach.empty()) r.gate_breach = breach;
    return r;
}

int main(int argc, char** argv) {
    Args a = parse(argc, argv, "bridge_shm_futex");
    double oh = measure_overhead_ns(a.bytes);
    std::vector<RunResult> runs;
    for (int k = 0; k < a.runs; ++k) {
        std::printf("[bridge] run %d/%d ...\n", k + 1, a.runs);
        runs.push_back(one_run(a));
    }
    return report(a, runs, oh);
}

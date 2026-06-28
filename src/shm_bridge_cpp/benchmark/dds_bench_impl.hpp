// dds_bench_impl.hpp — shared DDS pub/sub run logic for fastdds_bench and
// cyclonedds_bench. Identical code; the transport is selected purely by
// RMW_IMPLEMENTATION at launch, so the two binaries differ ONLY in their label
// (and therefore which RMW the launch script sets). This guarantees the DDS
// measurement logic is byte-identical between FastDDS and CycloneDDS.
#pragma once

#include "bench_common.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace dds_bench {

using Img = sensor_msgs::msg::Image;

inline bench::RunResult one_run(const bench::Args& a) {
    using namespace bench;
    std::atomic<bool> run{true};
    std::atomic<int> ready{0};
    std::vector<std::vector<double>> lat(a.subs);
    std::vector<uint64_t> rx(a.subs, 0), lost(a.subs, 0);

    std::vector<std::thread> rts;
    for (int i = 0; i < a.subs; ++i) {
        rts.emplace_back([&, i]() {
            if (a.pin) pin_to_core(a.base_core + 1 + i);
            auto node = std::make_shared<rclcpp::Node>("sub_" + std::to_string(i));
            uint32_t prev = 0; bool first = true;
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            auto sub = node->create_subscription<Img>(
                "bench_dds", qos, [&](std::shared_ptr<const Img> m) {
                    uint64_t sent; std::memcpy(&sent, m->data.data(), sizeof(sent));
                    double ms = (mono_ns() - sent) / 1e6;
                    uint32_t seq; std::memcpy(&seq, m->data.data() + 8, sizeof(seq));
                    if (!first && seq > prev + 1) lost[i] += (seq - prev - 1);
                    first = false; prev = seq; rx[i]++;
                    if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
                });
            rclcpp::executors::SingleThreadedExecutor ex;
            ex.add_node(node);
            ready.fetch_add(1);
            while (run.load(std::memory_order_relaxed))
                ex.spin_some(std::chrono::milliseconds(5));
        });
    }

    std::thread wt([&]() {
        if (a.pin) pin_to_core(a.base_core);
        auto node = std::make_shared<rclcpp::Node>("pub");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        auto pub = node->create_publisher<Img>("bench_dds", qos);
        Img msg; msg.height = 1; msg.width = (uint32_t)a.bytes; msg.encoding = "mono8";
        msg.step = (uint32_t)a.bytes; msg.data.resize(a.bytes, 0);
        while (ready.load() < a.subs) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const auto period = std::chrono::duration_cast<clk::duration>(
            std::chrono::duration<double>(1.0 / a.rate));
        auto t0 = clk::now(); uint32_t k = 0;
        while (run.load(std::memory_order_relaxed)) {
            uint64_t t = mono_ns(); k++;
            std::memcpy(msg.data.data(), &t, sizeof(t));
            std::memcpy(msg.data.data() + 8, &k, sizeof(k));
            pub->publish(msg);
            std::this_thread::sleep_until(t0 + k * period);   // no coordinated omission
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

inline int run_main(int argc, char** argv, const std::string& label) {
    rclcpp::init(argc, argv);
    bench::Args a = bench::parse(argc, argv, label);
    double oh = bench::measure_overhead_ns(a.bytes);
    std::vector<bench::RunResult> runs;
    for (int k = 0; k < a.runs; ++k) {
        std::printf("[%s] run %d/%d ...\n", label.c_str(), k + 1, a.runs);
        runs.push_back(one_run(a));
    }
    int rc = bench::report(a, runs, oh);
    rclcpp::shutdown();
    return rc;
}

}  // namespace dds_bench

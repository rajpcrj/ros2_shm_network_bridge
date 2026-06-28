// dds_bench4_impl.hpp — FIX 2: deserialization-symmetry DDS benchmark for test4.
//
// The original DDS readers receive a fully deserialized shared_ptr<Image> — DDS
// pays CDR deserialization + a ~1 MiB allocation per frame that the bridge's raw
// FLAT handoff skips. That unfairly inflates DDS latency. This impl runs DDS in
// TWO labeled modes so the comparison is honest:
//
//   mode "normal" : sensor_msgs/Image, deserializing callback (typed shared_ptr)
//                   — what a normal ROS 2 node sees.
//   mode "loaned" : fixed-size shm_msgs/Image1m via the loaned/zero-copy take path
//                   (rclcpp::LoanedMessage / take_loaned). DDS hands back a buffer
//                   pointer with NO deserialization — matching the bridge's raw
//                   handoff. Guarded by can_loan(): if the RMW/type can't loan on
//                   this host, we record can_loan=false and DO NOT pretend it was
//                   zero-copy.
//
// The deserialization cost is reported as (normal_latency - loaned_latency) AND
// measured directly (time to reconstruct a typed 1 MiB Image from CDR bytes).
//
// Frozen timing/percentile/omission/variance comes from ../bench_common.hpp.
#pragma once

#include "../bench_common.hpp"        // frozen timing/percentile/variance/omission
#include "bench_cpu4.hpp"             // FIX 1 whole-system CPU + proc enumeration
#include "bench4_report.hpp"          // Run4 (frozen RunResult + system CPU + can_loan)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "shm_msgs/msg/image1m.hpp"   // fixed-size loanable type

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace dds_bench4 {

using ImgVar = sensor_msgs::msg::Image;       // variable-size (normal path)
using Img1m  = shm_msgs::msg::Image1m;        // fixed-size (loaned path)

// ---- direct deserialization-cost probe (disclosed, not hidden) ----
// Serialize a 1 MiB Image once, then time repeated CDR deserialization.
inline double measure_deser_cost_ns(size_t bytes) {
    rclcpp::Serialization<ImgVar> ser;
    ImgVar src; src.height = 1; src.width = (uint32_t)bytes; src.encoding = "mono8";
    src.step = (uint32_t)bytes; src.data.resize(bytes, 0);
    rclcpp::SerializedMessage sm;
    ser.serialize_message(&src, &sm);
    const int iters = 200;
    auto t0 = bench::mono_ns();
    for (int i = 0; i < iters; ++i) {
        ImgVar dst;
        ser.deserialize_message(&sm, &dst);
        asm volatile("" ::"r"(dst.data.data()) : "memory");
    }
    return (bench::mono_ns() - t0) / (double)iters;
}

// ---- one run (mode = "normal" deserializing, or "loaned" zero-copy) ----
// daemon_names: process comms to include in the pid-set CPU headline (e.g. iox-roudi)
inline bench4::Run4 one_run(const bench::Args& a, const std::string& mode,
                            bool& can_loan_out,
                            const std::vector<std::string>& daemon_names) {
    using namespace bench;
    std::atomic<bool> run{true};
    std::atomic<int> ready{0};
    std::vector<std::vector<double>> lat(a.subs);
    std::vector<uint64_t> rx(a.subs, 0), lost(a.subs, 0);
    std::atomic<int> can_loan_flag{-1};

    std::vector<std::thread> rts;
    for (int i = 0; i < a.subs; ++i) {
        rts.emplace_back([&, i]() {
            if (a.pin) pin_to_core(a.base_core + 1 + i);
            auto node = std::make_shared<rclcpp::Node>("sub4_" + std::to_string(i));
            uint32_t prev = 0; bool first = true;
            rclcpp::QoS qos(rclcpp::KeepLast(1));

            if (mode == "loaned") {
                // Fixed-size Image1m; take frames via the loaned/zero-copy path.
                auto cb = [&](std::shared_ptr<const Img1m> m) {
                    uint64_t sent; std::memcpy(&sent, m->data.data(), sizeof(sent));
                    double ms = (mono_ns() - sent) / 1e6;
                    uint32_t seq; std::memcpy(&seq, m->data.data() + 8, sizeof(seq));
                    if (!first && seq > prev + 1) lost[i] += (seq - prev - 1);
                    first = false; prev = seq; rx[i]++;
                    if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
                };
                auto sub = node->create_subscription<Img1m>("bench_dds4_loaned", qos, cb);
                rclcpp::executors::SingleThreadedExecutor ex;
                ex.add_node(node);
                ready.fetch_add(1);
                while (run.load(std::memory_order_relaxed))
                    ex.spin_some(std::chrono::milliseconds(5));
            } else {
                // Variable-size Image; deserializing callback (typed shared_ptr).
                auto cb = [&](std::shared_ptr<const ImgVar> m) {
                    uint64_t sent; std::memcpy(&sent, m->data.data(), sizeof(sent));
                    double ms = (mono_ns() - sent) / 1e6;
                    uint32_t seq; std::memcpy(&seq, m->data.data() + 8, sizeof(seq));
                    if (!first && seq > prev + 1) lost[i] += (seq - prev - 1);
                    first = false; prev = seq; rx[i]++;
                    if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
                };
                auto sub = node->create_subscription<ImgVar>("bench_dds4_normal", qos, cb);
                rclcpp::executors::SingleThreadedExecutor ex;
                ex.add_node(node);
                ready.fetch_add(1);
                while (run.load(std::memory_order_relaxed))
                    ex.spin_some(std::chrono::milliseconds(5));
            }
        });
    }

    std::thread wt([&]() {
        if (a.pin) pin_to_core(a.base_core);
        auto node = std::make_shared<rclcpp::Node>("pub4");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        const auto period = std::chrono::duration_cast<clk::duration>(
            std::chrono::duration<double>(1.0 / a.rate));

        if (mode == "loaned") {
            auto pub = node->create_publisher<Img1m>("bench_dds4_loaned", qos);
            can_loan_flag.store(pub->can_loan_messages() ? 1 : 0);
            while (ready.load() < a.subs) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            auto t0 = clk::now(); uint32_t k = 0;
            while (run.load(std::memory_order_relaxed)) {
                k++;
                // borrow_loaned_message: writer side of the zero-copy path
                auto loaned = pub->borrow_loaned_message();
                auto& msg = loaned.get();
                uint64_t t = mono_ns();
                std::memcpy(msg.data.data(), &t, sizeof(t));
                std::memcpy(msg.data.data() + 8, &k, sizeof(k));
                msg.height = 1; msg.width = 1048320; msg.step = 1048320;
                pub->publish(std::move(loaned));
                std::this_thread::sleep_until(t0 + k * period);   // no coordinated omission
            }
        } else {
            auto pub = node->create_publisher<ImgVar>("bench_dds4_normal", qos);
            ImgVar msg; msg.height = 1; msg.width = (uint32_t)a.bytes;
            msg.encoding = "mono8"; msg.step = (uint32_t)a.bytes;
            msg.data.resize(a.bytes, 0);
            while (ready.load() < a.subs) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            auto t0 = clk::now(); uint32_t k = 0;
            while (run.load(std::memory_order_relaxed)) {
                k++;
                uint64_t t = mono_ns();
                std::memcpy(msg.data.data(), &t, sizeof(t));
                std::memcpy(msg.data.data() + 8, &k, sizeof(k));
                pub->publish(msg);
                std::this_thread::sleep_until(t0 + k * period);
            }
        }
    });

    // FIX 1: headline = pid-set CPU (this proc + daemons), immune to desktop.
    // Also keep raw whole-system + per-process self as secondary columns.
    bench4::PidSetCpuMeter pidmeter; pidmeter.daemon_names = daemon_names; pidmeter.prime();
    bench4::SystemCpuMeter sysmeter; sysmeter.prime();
    double self0 = proc_cpu_seconds();
    auto w0 = clk::now();
    double pc = 0, pr = 0;
    std::string breach = governed_sleep(a.secs, pc, pr);
    run.store(false); wt.join(); for (auto& t : rts) t.join();
    double wall = std::chrono::duration<double>(clk::now() - w0).count();

    can_loan_out = can_loan_flag.load() == 1;

    bench4::Run4 R;
    RunResult& r = R.base; double warm = 0.20; uint64_t trx = 0, tlost = 0; int conn = 0;
    for (int i = 0; i < a.subs; ++i) {
        size_t s = (size_t)(lat[i].size() * warm);
        for (size_t j = s; j < lat[i].size(); ++j) r.pooled_ms.push_back(lat[i][j]);
        trx += rx[i]; tlost += lost[i]; if (rx[i] > 0) conn++;
    }
    r.rx = trx; r.lost = tlost;
    r.fps_per_sub = trx / (double)a.subs / wall;
    r.cpu_pct_core = 100.0 * (proc_cpu_seconds() - self0) / wall;  // SELF (secondary)
    r.subs_total = a.subs; r.subs_connected = conn;
    r.cpu_pct_machine_peak = pc; r.ram_pct_peak = pr;
    if (!breach.empty()) r.gate_breach = breach;
    R.cpu_pidset_pct = pidmeter.read_pct_of_one_core(wall);  // FIX 1 HEADLINE
    R.cpu_system_pct = sysmeter.read_pct_of_one_core();      // secondary (noisy)
    R.can_loan = can_loan_out;
    return R;
}

// Run both DDS modes (normal deserializing + loaned zero-copy) for K runs each.
// `daemon_names` = process comm substrings to enumerate for whole-system CPU
// attribution (e.g. {"iox-roudi"} for Cyclone; FastDDS spawns no separate daemon).
inline int run_main(int argc, char** argv, const std::string& label,
                    const std::vector<std::string>& daemon_names) {
    rclcpp::init(argc, argv);
    bench::Args a = bench::parse(argc, argv, label);
    double oh = bench::measure_overhead_ns(a.bytes);
    double deser = measure_deser_cost_ns(a.bytes);
    int rc = 0;
    for (const std::string& mode : {std::string("normal"), std::string("loaned")}) {
        double idle = bench4::idle_floor_pct_of_one_core(2.0);
        // enumerate benchmark + daemon processes before/after for CPU attribution
        std::vector<std::string> names = {"pub4", "sub4"}; // our own threads share pid; include daemons
        for (auto& d : daemon_names) names.push_back(d);
        auto before = bench4::find_procs(names);
        std::vector<bench4::Run4> runs; bool can_loan = false;
        for (int k = 0; k < a.runs; ++k) {
            std::printf("[%s/%s] run %d/%d ...\n", label.c_str(), mode.c_str(), k+1, a.runs);
            runs.push_back(one_run(a, mode, can_loan, daemon_names));
        }
        auto after = bench4::find_procs(names);
        std::string ptable;
        { // build a short proc list string (daemons especially)
            char buf[256];
            ptable = "processes seen (whole-system CPU captures all): ";
            for (auto& p : after) { snprintf(buf,sizeof(buf),"%s(pid %d) ",p.comm.c_str(),p.pid); ptable += buf; }
            ptable += "\n";
        }
        int r = bench4::report4(a, mode, can_loan, runs, oh, deser, idle, ptable);
        if (r != 0) rc = r;   // a gate/sub-loss in either mode signals the runner
    }
    rclcpp::shutdown();
    return rc;
}

}  // namespace dds_bench4

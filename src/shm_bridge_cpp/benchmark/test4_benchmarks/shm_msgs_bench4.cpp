// shm_msgs_bench4.cpp — test4 benchmark for the upstream ros2_shm_msgs approach:
// the FIXED-SIZE Image ladder (Image8k/512k/1m/2m/4m/8m), the performance_test
// style of distinct fixed types. Two modes (symmetric with the DDS binaries):
//   loaned : borrow_loaned_message<ImageNk> (zero-copy, the ros2_shm_msgs path)
//   normal : deserializing subscription callback on the same ImageNk
// Headline CPU = pid-set (FIX 1). The --size arg picks the ladder rung; each rung
// is a distinct compile-time type so we dispatch with a template.
//
// Launch under a loan-capable RMW (rmw_fastrtps_cpp data-sharing, or cyclone+iox).
#include "../bench_common.hpp"
#include "bench_cpu4.hpp"
#include "bench4_report.hpp"

#include <rclcpp/rclcpp.hpp>
#include "shm_msgs/msg/image8k.hpp"
#include "shm_msgs/msg/image512k.hpp"
#include "shm_msgs/msg/image1m.hpp"
#include "shm_msgs/msg/image2m.hpp"
#include "shm_msgs/msg/image4m.hpp"
#include "shm_msgs/msg/image8m.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace bench;

// One run for a specific fixed-size message type T (loaned or normal mode).
template <typename T>
static bench4::Run4 run_typed(const Args& a_in, const std::string& mode,
                              const std::vector<std::string>& daemon_names,
                              bool& can_loan_out) {
    // CRITICAL: the fixed type T has a compile-time data array (e.g. Image8k =
    // uint8[16384]). Record the REAL payload size so the CSV/labels match what is
    // actually transmitted — NOT the default --bytes. T{}.data is std::array, so
    // .size() is the true on-wire payload. This is the size every other transport
    // must also run for the comparison to be apples-to-apples.
    Args a = a_in;
    a.bytes = T{}.data.size();
    std::atomic<bool> run{true};
    std::atomic<int> ready{0};
    std::atomic<int> can_loan_flag{-1};
    std::vector<std::vector<double>> lat(a.subs);
    std::vector<uint64_t> rx(a.subs, 0), lost(a.subs, 0);
    const std::string topic = "shm_msgs_bench4";

    std::vector<std::thread> rts;
    for (int i = 0; i < a.subs; ++i) {
        rts.emplace_back([&, i]() {
            if (a.pin) pin_to_core(a.base_core + 1 + i);
            auto node = std::make_shared<rclcpp::Node>("smsub4_" + std::to_string(i));
            uint32_t prev = 0; bool first = true;
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            // Both modes use a typed callback; the difference is the publisher:
            // loaned mode publishes via borrow_loaned (zero-copy on the wire),
            // normal mode publishes a normal copy (deserialized on receipt).
            auto cb = [&](std::shared_ptr<const T> m) {
                uint64_t sent; std::memcpy(&sent, m->data.data(), sizeof(sent));
                double ms = (mono_ns() - sent) / 1e6;
                uint32_t seq; std::memcpy(&seq, m->data.data() + 8, sizeof(seq));
                if (!first && seq > prev + 1) lost[i] += (seq - prev - 1);
                first = false; prev = seq; rx[i]++;
                if (ms >= 0 && ms < 60000) lat[i].push_back(ms);
            };
            auto sub = node->create_subscription<T>(topic, qos, cb);
            rclcpp::executors::SingleThreadedExecutor ex; ex.add_node(node);
            ready.fetch_add(1);
            while (run.load(std::memory_order_relaxed))
                ex.spin_some(std::chrono::milliseconds(5));
        });
    }

    std::thread wt([&]() {
        if (a.pin) pin_to_core(a.base_core);
        auto node = std::make_shared<rclcpp::Node>("smpub4");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        auto pub = node->create_publisher<T>(topic, qos);
        can_loan_flag.store(pub->can_loan_messages() ? 1 : 0);
        while (ready.load() < a.subs) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const auto period = std::chrono::duration_cast<clk::duration>(
            std::chrono::duration<double>(1.0 / a.rate));
        auto t0 = clk::now(); uint32_t k = 0;
        while (run.load(std::memory_order_relaxed)) {
            k++;
            if (mode == "loaned") {
                auto loaned = pub->borrow_loaned_message();   // zero-copy path
                auto& msg = loaned.get();
                uint64_t t = mono_ns();
                std::memcpy(msg.data.data(), &t, sizeof(t));
                std::memcpy(msg.data.data() + 8, &k, sizeof(k));
                pub->publish(std::move(loaned));
            } else {
                T msg;                                        // normal copy/deserialize
                uint64_t t = mono_ns();
                std::memcpy(msg.data.data(), &t, sizeof(t));
                std::memcpy(msg.data.data() + 8, &k, sizeof(k));
                pub->publish(msg);
            }
            std::this_thread::sleep_until(t0 + k * period);   // no coordinated omission
        }
    });

    bench4::PidSetCpuMeter pidmeter; pidmeter.daemon_names = daemon_names; pidmeter.prime();
    bench4::SystemCpuMeter sysmeter; sysmeter.prime();
    double self0 = proc_cpu_seconds(); auto w0 = clk::now();
    double pc = 0, pr = 0;
    std::string breach = governed_sleep(a.secs, pc, pr);
    run.store(false); wt.join(); for (auto& t : rts) t.join();
    double wall = std::chrono::duration<double>(clk::now() - w0).count();
    can_loan_out = can_loan_flag.load() == 1;

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
    R.cpu_pidset_pct = pidmeter.read_pct_of_one_core(wall);
    R.cpu_system_pct = sysmeter.read_pct_of_one_core();
    R.can_loan = can_loan_out;
    return R;
}

// True compile-time payload size (the type's data array) for a size label.
// These are the ACTUAL on-wire bytes — note Image8k is 16384, not 8192.
static size_t bytes_for_label(const std::string& s) {
    using namespace shm_msgs::msg;
    if (s=="8k")   return Image8k{}.data.size();    // 16384
    if (s=="512k") return Image512k{}.data.size();  // 524288
    if (s=="1m")   return Image1m{}.data.size();    // 1048576
    if (s=="2m")   return Image2m{}.data.size();    // 2097152
    if (s=="4m")   return Image4m{}.data.size();    // 4194304
    if (s=="8m")   return Image8m{}.data.size();    // 8388608
    return 0;
}

// Dispatch a run to the right fixed-size type by --size label, both modes.
static int run_size(const Args& a0, const std::string& size_label,
                    const std::vector<std::string>& daemons) {
    using namespace shm_msgs::msg;
    const size_t real_bytes = bytes_for_label(size_label);
    if (real_bytes == 0) { std::fprintf(stderr, "unknown --size %s\n", size_label.c_str()); return 1; }
    double oh = bench::measure_overhead_ns(real_bytes);   // overhead for the REAL size
    int rc = 0;
    for (const std::string& mode : {std::string("loaned"), std::string("normal")}) {
        Args a = a0;
        a.bytes = real_bytes;   // report the TRUE on-wire payload, not the --bytes default
        double idle = bench4::idle_floor_pct_of_one_core(2.0);
        std::vector<bench4::Run4> runs; bool can_loan = false;
        for (int k = 0; k < a.runs; ++k) {
            std::printf("[shm_msgs/%s/%s] run %d/%d ...\n", size_label.c_str(),
                        mode.c_str(), k+1, a.runs);
            bench4::Run4 R;
            if      (size_label=="8k")   R = run_typed<Image8k>  (a, mode, daemons, can_loan);
            else if (size_label=="512k") R = run_typed<Image512k>(a, mode, daemons, can_loan);
            else if (size_label=="1m")   R = run_typed<Image1m>  (a, mode, daemons, can_loan);
            else if (size_label=="2m")   R = run_typed<Image2m>  (a, mode, daemons, can_loan);
            else if (size_label=="4m")   R = run_typed<Image4m>  (a, mode, daemons, can_loan);
            else if (size_label=="8m")   R = run_typed<Image8m>  (a, mode, daemons, can_loan);
            else { std::fprintf(stderr, "unknown --size %s\n", size_label.c_str()); return 1; }
            runs.push_back(R);
        }
        a.transport = "ros2_shm_msgs_Image" + size_label;
        int r = bench4::report4(a, mode, can_loan, runs, oh, /*deser_ns=*/0.0, idle,
                                "processes seen: shm_msgs benchmark (+ daemons in pid-set)\n");
        if (r != 0) rc = r;
    }
    return rc;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    Args a = parse(argc, argv, "ros2_shm_msgs");
    // --size picks the ladder rung (8k/512k/1m/2m/4m/8m); default 1m.
    std::string size_label = "1m";
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--size" && i + 1 < argc) size_label = argv[i+1];
    // daemon names for the pid-set CPU headline (iox-roudi under cyclone; none for fast)
    std::vector<std::string> daemons;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--daemon" && i + 1 < argc) daemons.push_back(argv[i+1]);
    int rc = run_size(a, size_label, daemons);
    rclcpp::shutdown();
    return rc;
}

// bench4_report.hpp — test4 result type + reporter that carries BOTH CPU columns
// (self vs whole-system), the DDS mode, can_loan, idle floor, and the
// deserialization-cost disclosure. Wraps the frozen bench::RunResult; does not
// modify ../bench_common.hpp.
#pragma once

#include "../bench_common.hpp"
#include "bench_cpu4.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace bench4 {

// Per-run record = frozen RunResult + the test4 extras.
struct Run4 {
    bench::RunResult base;     // frozen fields (pooled_ms, fps, lost, cpu self, ...)
    double cpu_pidset_pct = 0; // FIX 1 HEADLINE: Σ CPU of bench pid + daemons
                               //   (counts iox-roudi; immune to desktop activity)
    double cpu_system_pct = 0; // secondary: raw whole-system CPU (disclosed, noisy)
    bool can_loan = false;     // FIX 2: did the loaned path actually loan?
};

inline double mean_of(const std::vector<double>& v){ return bench::mean_of(v); }
inline double sd_of(const std::vector<double>& v){ return bench::stdev_of(v); }

// Aggregate K runs and append a CSV row. Returns 0/2/3 like the frozen report():
// 2 = subscriber lost, 3 = CPU/RAM gate breach (so the runner can stop at the gate).
inline int report4(const bench::Args& a, const std::string& mode, bool can_loan,
                   std::vector<Run4>& runs, double overhead_ns, double deser_ns,
                   double idle_floor_pct, const std::string& proc_list) {
    auto across = [&](auto g){ std::vector<double> v; for(auto&r:runs)v.push_back(g(r));
                               return std::make_pair(mean_of(v), sd_of(v)); };
    auto P = [&](double p){ return across([&](Run4&r){ return bench::pct(r.base.pooled_ms,p);} ); };
    auto pmean = across([&](Run4&r){ return mean_of(r.base.pooled_ms); });
    auto p50=P(50), p90=P(90), p99=P(99), p999=P(99.9);
    auto pmax = across([&](Run4&r){ return r.base.pooled_ms.empty()?0.0:
                 *std::max_element(r.base.pooled_ms.begin(), r.base.pooled_ms.end()); });
    auto fps = across([&](Run4&r){ return r.base.fps_per_sub; });
    auto cpu_self   = across([&](Run4&r){ return r.base.cpu_pct_core; });
    auto cpu_pidset = across([&](Run4&r){ return r.cpu_pidset_pct; });
    auto cpu_sys    = across([&](Run4&r){ return r.cpu_system_pct; });
    uint64_t rx=0,lost=0; int min_conn=a.subs; double peak_cpu=0,peak_ram=0; std::string breach;
    for(auto&r:runs){ rx+=r.base.rx; lost+=r.base.lost; min_conn=std::min(min_conn,r.base.subs_connected);
        peak_cpu=std::max(peak_cpu,r.base.cpu_pct_machine_peak); peak_ram=std::max(peak_ram,r.base.ram_pct_peak);
        if(breach.empty()&&!r.base.gate_breach.empty()) breach=r.base.gate_breach; }
    double lostpct = (rx+lost)? 100.0*lost/(rx+lost):0.0;
    bool sub_lost = min_conn < a.subs;

    std::printf("\n=== %s [mode=%s can_loan=%s] (1 writer + %d reader threads) ===\n",
                a.transport.c_str(), mode.c_str(), can_loan?"yes":"no", a.subs);
    std::printf("payload=%zu B  rate=%.0f Hz  secs=%.0f  runs=%d  %s  pin=%s\n",
                a.bytes,a.rate,a.secs,a.runs,bench::env_note().c_str(),a.pin?"yes":"no");
    std::printf("idle whole-system CPU floor (pre-run) = %.1f%% of one core\n", idle_floor_pct);
    std::printf("measurement overhead ~= %.0f ns/frame; deser cost (1MiB CDR) = %.0f ns/frame\n",
                overhead_ns, deser_ns);
    std::printf("latency ms: mean=%.3f±%.3f p50=%.3f±%.3f p90=%.3f±%.3f p99=%.3f±%.3f p99.9=%.3f±%.3f max=%.3f±%.3f\n",
                pmean.first,pmean.second,p50.first,p50.second,p90.first,p90.second,
                p99.first,p99.second,p999.first,p999.second,pmax.first,pmax.second);
    std::printf("delivered: %.1f±%.1f fps/sub  lost=%.3f%%\n", fps.first,fps.second,lostpct);
    std::printf("CPU HEADLINE (pid-set: bench+daemons, immune to desktop): %.1f±%.1f %% of one core\n",
                cpu_pidset.first, cpu_pidset.second);
    std::printf("  CPU self (this process only): %.1f±%.1f %% of one core\n", cpu_self.first,cpu_self.second);
    std::printf("  CPU raw whole-system (noisy, incl. desktop): %.1f±%.1f %% of one core\n", cpu_sys.first,cpu_sys.second);
    if(!proc_list.empty()) std::printf("%s", proc_list.c_str());
    std::printf("HEALTH subs_connected=%d/%d peak_cpu=%.1f%% peak_ram=%.1f%% gate=%s sub_lost=%s\n",
                min_conn,a.subs,peak_cpu,peak_ram,breach.empty()?"none":breach.c_str(),sub_lost?"YES":"no");

    if(!a.csv.empty()){
        bool ex = std::ifstream(a.csv).good();
        std::ofstream f(a.csv, std::ios::app);
        if(!ex) f << "transport,mode,can_loan,subs,bytes,rate,secs,runs,"
                     "mean_ms,mean_sd,p50_ms,p50_sd,p90_ms,p90_sd,p99_ms,p99_sd,p999_ms,p999_sd,max_ms,max_sd,"
                     "fps_per_sub,fps_sd,lost_pct,cpu_pidset_pct,cpu_pidset_sd,cpu_self_pct,cpu_self_sd,"
                     "cpu_system_pct,cpu_system_sd,idle_floor_pct,overhead_ns,deser_ns,"
                     "subs_connected,subs_total,peak_ram,gate,sub_lost\n";
        f << a.transport<<","<<mode<<","<<(can_loan?1:0)<<","<<a.subs<<","<<a.bytes<<","<<a.rate<<","
          << a.secs<<","<<a.runs<<","<<pmean.first<<","<<pmean.second<<","<<p50.first<<","<<p50.second<<","
          << p90.first<<","<<p90.second<<","<<p99.first<<","<<p99.second<<","<<p999.first<<","<<p999.second<<","
          << pmax.first<<","<<pmax.second<<","<<fps.first<<","<<fps.second<<","<<lostpct<<","
          << cpu_pidset.first<<","<<cpu_pidset.second<<","<<cpu_self.first<<","<<cpu_self.second<<","
          << cpu_sys.first<<","<<cpu_sys.second<<","
          << idle_floor_pct<<","<<overhead_ns<<","<<deser_ns<<","<<min_conn<<","<<a.subs<<","
          << peak_ram<<","<<(breach.empty()?"none":breach)<<","<<(sub_lost?1:0)<<"\n";
    }
    if(!breach.empty()) return 3;
    if(sub_lost) return 2;
    return 0;
}

}  // namespace bench4

// cyclonedds_bench4.cpp — test4 DDS benchmark under CycloneDDS+iceoryx. Runs BOTH
// modes (normal + loaned). The "iox-roudi" daemon is enumerated so the
// whole-system CPU metric (FIX 1) provably captures it — the original self-CPU
// metric missed roudi entirely. Launch with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
// + CYCLONEDDS_URI=...cyclonedds_shm.xml and iox-roudi already running.
#include "dds_bench4_impl.hpp"

int main(int argc, char** argv) {
    return dds_bench4::run_main(argc, argv, "cyclonedds_iceoryx", /*daemons=*/{"iox-roudi"});
}

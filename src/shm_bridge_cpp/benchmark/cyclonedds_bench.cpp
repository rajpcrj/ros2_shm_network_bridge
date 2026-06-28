// cyclonedds_bench.cpp — DDS benchmark under CycloneDDS+iceoryx. Launch with
//   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp + CYCLONEDDS_URI=...cyclonedds_shm.xml
//   and iox-roudi running. Run logic shared with fastdds_bench (dds_bench_impl.hpp).
#include "dds_bench_impl.hpp"

int main(int argc, char** argv) {
    return dds_bench::run_main(argc, argv, "cyclonedds_iceoryx");
}

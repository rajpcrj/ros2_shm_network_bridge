// fastdds_bench.cpp — DDS benchmark under FastDDS. Launch with
//   RMW_IMPLEMENTATION=rmw_fastrtps_cpp (+ data-sharing XML). All run logic is in
// dds_bench_impl.hpp so it is byte-identical to cyclonedds_bench.
#include "dds_bench_impl.hpp"

int main(int argc, char** argv) {
    return dds_bench::run_main(argc, argv, "fastdds_datasharing");
}

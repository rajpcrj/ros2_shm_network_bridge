// fastdds_bench4.cpp — test4 DDS benchmark under FastDDS. Runs BOTH modes
// (normal deserializing + loaned zero-copy) with whole-system CPU accounting.
// FastDDS data-sharing spawns no separate daemon, so its whole-system CPU ≈ self
// CPU (the honest check that the metric is symmetric). Launch with
// RMW_IMPLEMENTATION=rmw_fastrtps_cpp + the data-sharing XML.
#include "dds_bench4_impl.hpp"

int main(int argc, char** argv) {
    return dds_bench4::run_main(argc, argv, "fastdds_datasharing", /*daemons=*/{});
}

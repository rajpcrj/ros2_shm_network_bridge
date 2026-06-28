cmake_minimum_required(VERSION 3.8)
project(shm_bridge_cpp)

# =========================
# C++ standard
# =========================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# =========================
# Compiler settings
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -O3
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# =========================
# Find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# heavy-type message packages for the C++ FLAT adapter (ros2_to_shm_flat)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmw_dds_common REQUIRED)

# =========================
# LIBRARY: libshm_bridge_cpp.so
# Reusable Writer/Reader over the SHM contract. Other packages do:
#   find_package(shm_bridge_cpp REQUIRED)
#   target_link_libraries(my_node shm_bridge_cpp::shm_bridge_cpp)
#   #include <shm_bridge_cpp/shm_bridge.hpp>
# =========================
add_library(shm_bridge_cpp SHARED
  src/shm_bridge.cpp
)
target_include_directories(shm_bridge_cpp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# export the target under the package namespace for downstream find_package
ament_export_targets(shm_bridge_cppTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS shm_bridge_cpp
  EXPORT shm_bridge_cppTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# =========================
# Program 1:
# ROS2 -> Shared Memory (RGB + Depth Heatmap)
# =========================
add_executable(ros_rgb_depth_to_shm
  src/ros_rgb_depth_to_shm.cpp
)

ament_target_dependencies(ros_rgb_depth_to_shm
  rclcpp
  sensor_msgs
)

target_include_directories(ros_rgb_depth_to_shm PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ros_rgb_depth_to_shm
  ${OpenCV_LIBS}
)

# =========================
# Program 2:
# Shared Memory -> Viewer (OpenCV, no ROS)
# =========================
add_executable(shm_rgb_depth_viewer
  src/shm_rgb_depth_viewer.cpp
)

target_include_directories(shm_rgb_depth_viewer PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(shm_rgb_depth_viewer
  ${OpenCV_LIBS}
)

# =========================
# Program 3:
# Generic type-agnostic ROS2 -> SHM writer (CDR via GenericSubscription)
# =========================
add_executable(generic_shm_writer
  src/generic_shm_writer.cpp
)

ament_target_dependencies(generic_shm_writer
  rclcpp
)

target_include_directories(generic_shm_writer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Program 4:
# Generic type-agnostic SHM reader (no ROS deps; pure contract)
# =========================
add_executable(generic_shm_reader
  src/generic_shm_reader.cpp
)

target_include_directories(generic_shm_reader PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Program 5:
# ros2_to_shm — one topic per process, auto-resolve type, universal CDR
# =========================
add_executable(ros2_to_shm
  src/ros2_to_shm.cpp
)

ament_target_dependencies(ros2_to_shm
  rclcpp
)

target_include_directories(ros2_to_shm PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Program 6:
# ros2_to_shm_flat — typed zero-copy FLAT for the 25 heavy types + CDR fallback
# =========================
add_executable(ros2_to_shm_flat
  src/ros2_to_shm_flat.cpp
)

ament_target_dependencies(ros2_to_shm_flat
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  stereo_msgs
  map_msgs
  octomap_msgs
  visualization_msgs
  rmw_dds_common
)

target_include_directories(ros2_to_shm_flat PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Example: how to use the library (links libshm_bridge_cpp.so)
# =========================
add_executable(example_usage
  examples/example_usage.cpp
)
target_link_libraries(example_usage
  shm_bridge_cpp
)

# =========================
# Benchmarks (single-process, perf_test-style; link libshm_bridge_cpp.so)
# =========================
# Bridge benchmark: pure library + futex, no ROS.
add_executable(bridge_bench benchmark/bridge_bench.cpp)
target_include_directories(bridge_bench PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/benchmark
)
target_link_libraries(bridge_bench shm_bridge_cpp pthread)

# DDS benchmarks: rclcpp pub/sub, transport chosen by RMW at launch.
add_executable(fastdds_bench benchmark/fastdds_bench.cpp)
target_include_directories(fastdds_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/benchmark)
ament_target_dependencies(fastdds_bench rclcpp sensor_msgs)
target_link_libraries(fastdds_bench pthread)

add_executable(cyclonedds_bench benchmark/cyclonedds_bench.cpp)
target_include_directories(cyclonedds_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/benchmark)
ament_target_dependencies(cyclonedds_bench rclcpp sensor_msgs)
target_link_libraries(cyclonedds_bench pthread)

# ----- test4: bridge with whole-system CPU accounting (FIX 1) + size-ladder (FIX 3) -----
# DDS test4 binaries (FIX 2 loaned path) live in the loan_bench package because
# they need the fixed-size shm_msgs/Image1m type.
add_executable(bridge_bench4 benchmark/test4_benchmarks/bridge_bench4.cpp)
target_include_directories(bridge_bench4 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/benchmark
  ${CMAKE_CURRENT_SOURCE_DIR}/benchmark/test4_benchmarks
)
ament_target_dependencies(bridge_bench4 rclcpp sensor_msgs)  # deserialize mode builds Image
target_link_libraries(bridge_bench4 shm_bridge_cpp pthread)

# =========================
# Top-level examples/ programs (docs-facing, copy-paste starters).
# The pure-library / POSIX ones (write_cpp, read_cpp, shm_to_network) build with
# just the library; the ROS bridges (ros2_to_shm, shm_to_ros2) need rclcpp.
# =========================
set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/cpp)

add_executable(ex_write_cpp ${EXAMPLES_DIR}/write_cpp.cpp)
target_link_libraries(ex_write_cpp shm_bridge_cpp pthread)

add_executable(ex_read_cpp ${EXAMPLES_DIR}/read_cpp.cpp)
target_link_libraries(ex_read_cpp shm_bridge_cpp pthread)

add_executable(ex_shm_to_network ${EXAMPLES_DIR}/shm_to_network.cpp)
target_link_libraries(ex_shm_to_network shm_bridge_cpp pthread)

add_executable(ex_shm_to_udp ${EXAMPLES_DIR}/shm_to_udp.cpp)
target_link_libraries(ex_shm_to_udp shm_bridge_cpp pthread)

add_executable(ex_ros2_to_shm ${EXAMPLES_DIR}/ros2_to_shm.cpp)
ament_target_dependencies(ex_ros2_to_shm rclcpp sensor_msgs)
target_link_libraries(ex_ros2_to_shm shm_bridge_cpp pthread)

add_executable(ex_shm_to_ros2 ${EXAMPLES_DIR}/shm_to_ros2.cpp)
ament_target_dependencies(ex_shm_to_ros2 rclcpp sensor_msgs)
target_link_libraries(ex_shm_to_ros2 shm_bridge_cpp pthread)

# =========================
# Install targets
# =========================
install(
  TARGETS
    ros_rgb_depth_to_shm
    shm_rgb_depth_viewer
    generic_shm_writer
    generic_shm_reader
    ros2_to_shm
    ros2_to_shm_flat
    example_usage
    bridge_bench
    fastdds_bench
    cyclonedds_bench
    bridge_bench4
    ex_write_cpp
    ex_read_cpp
    ex_shm_to_network
    ex_shm_to_udp
    ex_ros2_to_shm
    ex_shm_to_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Testing / linting
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# Package export
# =========================
ament_package()

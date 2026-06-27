// flat_adapter.hpp — typed FLAT (zero-copy) extractors for the 25 heavy types
// (src/message_type/HEAVY_FLAT.txt). Mirror of adapter.py's FLAT_EXTRACTORS.
//
// Each extractor returns a Flat{ptr,len,width,height,channels,dtype_id} pointing
// at the message's existing buffer — NO copy. The writer memcpy's it into SHM once.
#pragma once

#include "shm_bridge_cpp/shm_contract.hpp"

#include <cstdint>
#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/mesh_file.hpp>
#include <rmw_dds_common/msg/gid.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace flat_adapter {

// dtype ids match shm_contract / adapter.py
enum : uint32_t {
    D_U8 = 0, D_I8 = 1, D_U16 = 2, D_I16 = 3,
    D_U32 = 4, D_I32 = 5, D_F32 = 6, D_F64 = 7
};

struct Flat {
    const void* ptr = nullptr;   // zero-copy pointer into the message buffer
    size_t len = 0;              // bytes
    uint32_t width = 0, height = 0, channels = 0;
    uint32_t dtype_id = D_U8;
    bool ok = false;
};

// Helper: a 1-D primitive array view -> Flat (len in bytes).
template <typename T>
inline Flat vec(const std::vector<T>& v, uint32_t dtype_id) {
    Flat f;
    f.ptr = v.data();
    f.len = v.size() * sizeof(T);
    f.width = static_cast<uint32_t>(v.size());
    f.height = 1; f.channels = 1; f.dtype_id = dtype_id; f.ok = true;
    return f;
}

// ---- per-type extractors ----

inline Flat image(const sensor_msgs::msg::Image& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = m.width; f.height = m.height;
    f.channels = (m.height && m.width) ?
        static_cast<uint32_t>(m.data.size() / (static_cast<size_t>(m.height) * m.width)) : 1;
    if (f.channels == 0) f.channels = 1;
    f.dtype_id = D_U8; f.ok = true; return f;
}

inline Flat compressed_image(const sensor_msgs::msg::CompressedImage& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = static_cast<uint32_t>(m.data.size()); f.height = 1; f.channels = 1;
    f.dtype_id = D_U8; f.ok = true; return f;
}

inline Flat pointcloud2(const sensor_msgs::msg::PointCloud2& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = m.width; f.height = m.height ? m.height : 1;
    f.channels = m.point_step; f.dtype_id = D_U8; f.ok = true; return f;
}

inline Flat laserscan(const sensor_msgs::msg::LaserScan& m) {
    return vec(m.ranges, D_F32);
}
inline Flat multiecho_laserscan(const sensor_msgs::msg::MultiEchoLaserScan& m) {
    // echoes are nested; fall back to first echo set if present, else empty
    Flat f; f.ok = false; (void)m; return f;  // not flat-friendly -> CDR
}
inline Flat disparity(const stereo_msgs::msg::DisparityImage& m) {
    return image(m.image);
}
inline Flat occupancy_grid(const nav_msgs::msg::OccupancyGrid& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = m.info.width; f.height = m.info.height; f.channels = 1;
    f.dtype_id = D_I8; f.ok = true; return f;
}
inline Flat occupancy_grid_update(const map_msgs::msg::OccupancyGridUpdate& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = m.width; f.height = m.height; f.channels = 1;
    f.dtype_id = D_I8; f.ok = true; return f;
}
inline Flat octomap(const octomap_msgs::msg::Octomap& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = static_cast<uint32_t>(m.data.size()); f.height = 1; f.channels = 1;
    f.dtype_id = D_I8; f.ok = true; return f;
}
inline Flat mesh_file(const visualization_msgs::msg::MeshFile& m) {
    return vec(m.data, D_U8);
}
inline Flat gid(const rmw_dds_common::msg::Gid& m) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size();
    f.width = static_cast<uint32_t>(m.data.size()); f.height = 1; f.channels = 1;
    f.dtype_id = D_U8; f.ok = true; return f;
}

// std_msgs MultiArray family: dominant 'data' array; shape from layout.dim.
template <typename Msg, typename T>
inline Flat multiarray(const Msg& m, uint32_t dtype_id) {
    Flat f; f.ptr = m.data.data(); f.len = m.data.size() * sizeof(T);
    const auto& dim = m.layout.dim;
    uint32_t w = dim.size() >= 1 ? dim[dim.size() - 1].size
                                 : static_cast<uint32_t>(m.data.size());
    uint32_t h = dim.size() >= 2 ? dim[dim.size() - 2].size : 1;
    uint32_t c = dim.size() >= 3 ? dim[dim.size() - 3].size : 1;
    f.width = w; f.height = h; f.channels = c; f.dtype_id = dtype_id; f.ok = true;
    return f;
}

// Is this type name handled by a typed FLAT extractor here?
inline bool is_flat(const std::string& t) {
    static const std::vector<std::string> names = {
        "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/LaserScan",
        "stereo_msgs/msg/DisparityImage", "nav_msgs/msg/OccupancyGrid",
        "map_msgs/msg/OccupancyGridUpdate", "octomap_msgs/msg/Octomap",
        "visualization_msgs/msg/MeshFile", "rmw_dds_common/msg/Gid",
        "std_msgs/msg/ByteMultiArray", "std_msgs/msg/UInt8MultiArray",
        "std_msgs/msg/Int8MultiArray", "std_msgs/msg/UInt16MultiArray",
        "std_msgs/msg/Int16MultiArray", "std_msgs/msg/UInt32MultiArray",
        "std_msgs/msg/Int32MultiArray", "std_msgs/msg/UInt64MultiArray",
        "std_msgs/msg/Int64MultiArray", "std_msgs/msg/Float32MultiArray",
        "std_msgs/msg/Float64MultiArray",
    };
    for (const auto& n : names) if (n == t) return true;
    return false;
}

}  // namespace flat_adapter

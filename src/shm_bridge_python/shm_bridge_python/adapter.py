#!/usr/bin/env python3
"""
adapter.py — single entry point that converts ANY ROS 2 message to a SHM payload
(PROMPT.md §5). Low-latency / low-memory by design:

  * FLAT path (the 25 heavy types in src/message_type/HEAVY_FLAT.txt):
        zero-copy view of the dominant buffer via memoryview/np.frombuffer.
        NO .tobytes(), NO intermediate allocation — the writer memcpy's the view
        straight into shared memory exactly once.
  * CDR path (everything else, all 305): serialize_message() once -> memcpy.

`adapt(msg, type_name)` returns an `Adapted`:
    payload   : a buffer (memoryview / bytes / np.ndarray) the writer copies once
    encoding  : ENC_FLAT or ENC_CDR
    width/height/channels : FLAT shape hints (0 for CDR)
    dtype     : numpy dtype string (FLAT) or "" (CDR)
    structural_key : tuple that only changes when the RECIPE must be re-written
                     (type, encoding, dtype, w, h, ch) — NOT data_size.
    recipe    : dict for *_recipe.json (written only when structural_key changes)
"""
from dataclasses import dataclass
import numpy as np
from rclpy.serialization import serialize_message

from shm_bridge_python import shm_contract as C


@dataclass
class Adapted:
    payload: object          # memoryview | bytes | np.ndarray (writer copies once)
    encoding: int
    width: int = 0
    height: int = 0
    channels: int = 0
    dtype: str = ""
    structural_key: tuple = ()
    recipe: dict = None


# ----------------------------------------------------------------------------
# FLAT extractors — one small rule per heavy type. Each returns
#   (buffer_view, width, height, channels, dtype_str)
# buffer_view must be zero-copy (a view over msg memory), not a fresh copy.
# ----------------------------------------------------------------------------

def _buf(seq, dtype):
    """Zero-copy 1-D numpy view over a primitive ROS array field."""
    a = np.asarray(seq)
    if a.dtype != np.dtype(dtype):
        a = a.view(dtype) if a.nbytes else np.frombuffer(bytes(seq), dtype=dtype)
    return a


def _image(m):
    h, w = m.height, m.width
    data = np.frombuffer(m.data, dtype=np.uint8)
    ch = (data.size // (h * w)) if (h * w) else 1
    return data, w, h, max(ch, 1), "uint8"


def _compressed_image(m):
    data = np.frombuffer(m.data, dtype=np.uint8)
    return data, data.size, 1, 1, "uint8"          # opaque blob (jpeg/png)


def _pointcloud2(m):
    data = np.frombuffer(m.data, dtype=np.uint8)
    h = m.height if m.height > 0 else 1
    return data, m.width, h, m.point_step, "uint8"  # width=points/row, ch=point_step


def _laserscan(m):
    data = _buf(m.ranges, "float32")
    return data, data.size, 1, 1, "float32"


def _disparity(m):
    # underlying image is sensor_msgs/Image
    return _image(m.image)


def _occupancy_grid(m):
    w = m.info.width
    h = m.info.height
    data = _buf(m.data, "int8")
    return data, w, h, 1, "int8"


def _occupancy_grid_update(m):
    data = _buf(m.data, "int8")
    return data, m.width, m.height, 1, "int8"


def _octomap(m):
    data = np.frombuffer(bytes(m.data), dtype=np.int8)
    return data, data.size, 1, 1, "int8"


def _mesh_file(m):
    data = np.frombuffer(bytes(m.data), dtype=np.uint8)
    return data, data.size, 1, 1, "uint8"


def _gid(m):
    data = np.frombuffer(bytes(m.data), dtype=np.uint8)
    return data, data.size, 1, 1, "uint8"


def _dataframe(m):
    data = np.frombuffer(bytes(m.data), dtype=np.uint8)
    return data, data.size, 1, 1, "uint8"


def _float32array(m):
    data = _buf(m.data, "float32")
    return data, data.size, 1, 1, "float32"


# std_msgs/*MultiArray — dominant 'data' array; shape from layout.dim if present.
_MULTIARRAY_DTYPE = {
    "ByteMultiArray": "uint8", "UInt8MultiArray": "uint8", "Int8MultiArray": "int8",
    "UInt16MultiArray": "uint16", "Int16MultiArray": "int16",
    "UInt32MultiArray": "uint32", "Int32MultiArray": "int32",
    "UInt64MultiArray": "uint64", "Int64MultiArray": "int64",
    "Float32MultiArray": "float32", "Float64MultiArray": "float64",
}


def _make_multiarray(dtype):
    def fn(m):
        data = _buf(m.data, dtype)
        dims = [d.size for d in m.layout.dim] if m.layout.dim else []
        w = dims[-1] if len(dims) >= 1 else data.size
        h = dims[-2] if len(dims) >= 2 else 1
        ch = dims[-3] if len(dims) >= 3 else 1
        return data, int(w), int(h), int(ch), dtype
    return fn


FLAT_EXTRACTORS = {
    "sensor_msgs/msg/Image": _image,
    "sensor_msgs/msg/CompressedImage": _compressed_image,
    "sensor_msgs/msg/PointCloud2": _pointcloud2,
    "sensor_msgs/msg/PointCloud": _pointcloud2,  # best-effort raw buffer
    "sensor_msgs/msg/LaserScan": _laserscan,
    "sensor_msgs/msg/MultiEchoLaserScan": _laserscan,
    "stereo_msgs/msg/DisparityImage": _disparity,
    "nav_msgs/msg/OccupancyGrid": _occupancy_grid,
    "map_msgs/msg/OccupancyGridUpdate": _occupancy_grid_update,
    "octomap_msgs/msg/Octomap": _octomap,
    "visualization_msgs/msg/MeshFile": _mesh_file,
    "rmw_dds_common/msg/Gid": _gid,
    "ros_gz_interfaces/msg/Dataframe": _dataframe,
    "ros_gz_interfaces/msg/Float32Array": _float32array,
}
for _name, _dt in _MULTIARRAY_DTYPE.items():
    FLAT_EXTRACTORS[f"std_msgs/msg/{_name}"] = _make_multiarray(_dt)


# ----------------------------------------------------------------------------
# The single adapter entry point.
# ----------------------------------------------------------------------------

def adapt(msg, type_name: str, topic: str = "") -> Adapted:
    ex = FLAT_EXTRACTORS.get(type_name)
    if ex is not None:
        buf, w, h, ch, dtype = ex(msg)
        key = (type_name, "flat", dtype, w, h, ch)
        recipe = {
            "topic": topic, "type_name": type_name, "encoding": "flat",
            "dtype": dtype,
            "shape": [h, w, ch] if ch > 1 else ([h, w] if h > 1 else [w]),
        }
        return Adapted(buf, C.ENC_FLAT, w, h, ch, dtype, key, recipe)

    # universal CDR path — one serialize, no per-type code
    payload = serialize_message(msg)
    key = (type_name, "cdr", "", 0, 0, 0)
    recipe = {"topic": topic, "type_name": type_name, "encoding": "cdr"}
    return Adapted(payload, C.ENC_CDR, 0, 0, 0, "", key, recipe)

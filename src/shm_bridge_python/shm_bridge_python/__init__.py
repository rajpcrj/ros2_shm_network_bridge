"""shm_bridge_python — generic ROS 2 <-> shared-memory transport.

Importable API (after `colcon build` + sourcing the workspace):

    from shm_bridge_python import StreamReader, adapt, shm_paths
    from shm_bridge_python import shm_contract as C

Building blocks:
    shm_contract  — byte-exact on-disk layout (header/frame/recipe, seqlock)
    adapt         — single function: msg -> FLAT (zero-copy) or CDR payload
    StreamReader  — type-agnostic reader (reconstructs ndarray or ROS message)
"""
from shm_bridge_python import shm_contract  # noqa: F401
from shm_bridge_python.shm_contract import shm_paths  # noqa: F401
from shm_bridge_python.adapter import adapt, Adapted  # noqa: F401
from shm_bridge_python.generic_shm_reader import StreamReader  # noqa: F401

__all__ = ["shm_contract", "shm_paths", "adapt", "Adapted", "StreamReader"]

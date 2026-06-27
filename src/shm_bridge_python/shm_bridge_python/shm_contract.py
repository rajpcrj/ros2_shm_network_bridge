#!/usr/bin/env python3
"""
shm_contract.py — the byte-exact on-disk contract (PROMPT.md §4).

Single source of truth for the binary header layout + seqlock, shared by the
generic writer and reader. The C++ side mirrors these exact offsets.

Per stream NAME, three files in /dev/shm:
  NAME_header        64-byte binary header (this module)
  NAME_frame         raw payload (FLAT numeric buffer OR CDR serialized bytes)
  NAME_recipe.json   human-readable decode recipe
"""
import json
import mmap
import os
import struct

HEADER_SIZE = 64

# encoding_id
ENC_FLAT = 0
ENC_CDR = 1

# dtype_id table (FLAT path) <-> numpy dtype string
DTYPE_BY_ID = {
    0: "uint8", 1: "int8", 2: "uint16", 3: "int16",
    4: "uint32", 5: "int32", 6: "float32", 7: "float64",
}
ID_BY_DTYPE = {v: k for k, v in DTYPE_BY_ID.items()}

# struct layout for the fixed part (offsets 0..39), little-endian:
#   seq, encoding_id, data_size, width, height, channels, dtype_id, reserved : 8x uint32
#   timestamp_ns : uint64
# followed by char[24] type_name (offsets 40..63).
_FIXED = struct.Struct("<8I Q")          # 32 (8*4) + 8 = 40 bytes
_TYPE_NAME_OFF = 40
_TYPE_NAME_LEN = 24
assert _FIXED.size == _TYPE_NAME_OFF
assert _TYPE_NAME_OFF + _TYPE_NAME_LEN == HEADER_SIZE


def shm_paths(name: str):
    base = f"/dev/shm/{name}"
    return f"{base}_header", f"{base}_frame", f"{base}_recipe.json"


def open_mmap(path: str, size: int, create: bool):
    flags = os.O_RDWR | (os.O_CREAT if create else 0)
    fd = os.open(path, flags, 0o666)
    if create:
        # only grow; never shrink an existing live buffer
        cur = os.fstat(fd).st_size
        if cur < size:
            os.ftruncate(fd, size)
        size = max(cur, size)
    else:
        size = os.fstat(fd).st_size
    prot = mmap.PROT_READ | (mmap.PROT_WRITE if create else 0)
    mm = mmap.mmap(fd, size, mmap.MAP_SHARED, prot)
    return fd, mm, size


def pack_header(seq, encoding_id, data_size, width, height,
                channels, dtype_id, timestamp_ns, type_name):
    buf = bytearray(HEADER_SIZE)
    _FIXED.pack_into(buf, 0, seq, encoding_id, data_size, width, height,
                     channels, dtype_id, 0, timestamp_ns)
    tn = type_name.encode("ascii", "replace")[:_TYPE_NAME_LEN]
    buf[_TYPE_NAME_OFF:_TYPE_NAME_OFF + len(tn)] = tn
    return bytes(buf)


def read_header(mm):
    raw = mm[0:HEADER_SIZE]
    (seq, encoding_id, data_size, width, height,
     channels, dtype_id, _reserved, timestamp_ns) = _FIXED.unpack_from(raw, 0)
    tn = raw[_TYPE_NAME_OFF:_TYPE_NAME_OFF + _TYPE_NAME_LEN].split(b"\x00", 1)[0]
    return {
        "seq": seq, "encoding_id": encoding_id, "data_size": data_size,
        "width": width, "height": height, "channels": channels,
        "dtype_id": dtype_id, "timestamp_ns": timestamp_ns,
        "type_name": tn.decode("ascii", "replace"),
    }


def read_seq(mm):
    """Atomic-ish single read of the seqlock counter (offset 0, uint32)."""
    return struct.unpack_from("<I", mm, 0)[0]


def write_recipe(path, recipe: dict):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(recipe, f)
    os.replace(tmp, path)  # atomic swap so readers never see a half-written recipe

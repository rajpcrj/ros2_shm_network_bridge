#!/usr/bin/env python3
"""
generate_index.py — populate src/message_type/ from the local ROS install.

Dumps every installed message schema (.msg) to src/message_type/<pkg>/<Name>.msg
and writes index.json mapping "pkg/msg/Name" -> {encoding_hint, fields}.

The FLAT/CDR encoding_hint is a *hint* the writer may use to pick the zero-copy
fast path; CDR is always a valid fallback (see PROMPT.md §5).

Run:  python3 src/message_type/generate_index.py
(no ROS sourcing required — it just reads files under the share tree)
"""
import json
import os
import re
import sys
from pathlib import Path

# Where ROS installs schemas. Override with ROS_SHARE env if needed.
ROS_SHARE = Path(os.environ.get("ROS_SHARE", "/opt/ros/humble/share"))
OUT_DIR = Path(__file__).resolve().parent

# Primitive scalar types that count toward a "flat numeric" message.
PRIMITIVE_NUMERIC = {
    "uint8", "int8", "uint16", "int16", "uint32", "int32",
    "uint64", "int64", "float32", "float64", "byte", "char",
}

# A message is FLAT-eligible (zero-copy candidate) if it is dominated by a single
# large primitive array (e.g. Image.data, *MultiArray.data). This is only a hint;
# the writer makes the final call. Conservative allowlist by type name + a
# structural check below.
FLAT_NAME_ALLOWLIST = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
    "std_msgs/msg/UInt8MultiArray",
    "std_msgs/msg/Int8MultiArray",
    "std_msgs/msg/UInt16MultiArray",
    "std_msgs/msg/Int16MultiArray",
    "std_msgs/msg/UInt32MultiArray",
    "std_msgs/msg/Int32MultiArray",
    "std_msgs/msg/Float32MultiArray",
    "std_msgs/msg/Float64MultiArray",
}

FIELD_RE = re.compile(r"^\s*([A-Za-z0-9_/]+)(\[[^\]]*\])?\s+([A-Za-z0-9_]+)")


def parse_fields(text: str):
    """Very small .msg field parser: returns [(type, is_array, name), ...].
    Skips comments, constants (lines with '='), and blank lines."""
    fields = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if "=" in line:  # constant definition, not a field
            continue
        m = FIELD_RE.match(line)
        if not m:
            continue
        ftype, arr, fname = m.group(1), m.group(2), m.group(3)
        fields.append({"type": ftype, "array": arr is not None, "name": fname})
    return fields


def flat_hint(type_name: str, fields) -> bool:
    if type_name in FLAT_NAME_ALLOWLIST:
        return True
    # Structural fallback: exactly one array field and it is a numeric primitive,
    # everything else scalar primitives/headers.
    arrays = [f for f in fields if f["array"]]
    if len(arrays) != 1:
        return False
    a = arrays[0]
    base = a["type"].split("/")[-1]
    return base in PRIMITIVE_NUMERIC


def main():
    if not ROS_SHARE.is_dir():
        print(f"ERROR: ROS share dir not found: {ROS_SHARE}", file=sys.stderr)
        return 1

    index = {}
    n_files = 0
    for msg_path in sorted(ROS_SHARE.glob("*/msg/*.msg")):
        pkg = msg_path.parts[-3]
        name = msg_path.stem
        type_name = f"{pkg}/msg/{name}"
        text = msg_path.read_text(errors="replace")

        # mirror the schema into src/message_type/<pkg>/<Name>.msg
        dst = OUT_DIR / pkg / f"{name}.msg"
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(text)
        n_files += 1

        fields = parse_fields(text)
        index[type_name] = {
            "encoding_hint": "flat" if flat_hint(type_name, fields) else "cdr",
            "fields": fields,
        }

    (OUT_DIR / "index.json").write_text(json.dumps(index, indent=2, sort_keys=True))
    flat = sum(1 for v in index.values() if v["encoding_hint"] == "flat")
    print(f"[message_type] dumped {n_files} schemas, {len(index)} types "
          f"({flat} flat / {len(index) - flat} cdr) -> {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

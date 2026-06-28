#!/usr/bin/env python3
"""
network_to_shm.py — receive the UDP stream from shm_to_network.cpp and write it
back into a LOCAL /dev/shm stream on this (remote) machine.

This completes the cross-machine path:
   machine A:  ros2_to_shm  ->  /dev/shm  ->  shm_to_network.cpp  --UDP-->
   machine B:  network_to_shm.py  ->  /dev/shm  ->  (read_py / shm_to_ros2 / ...)

It reassembles fragmented frames by (seq) using the 32-byte wire header defined in
shm_to_network.cpp, then publishes them under the same seqlock contract so any
local reader on machine B sees them as a normal SHM stream.

Run (receiver):  python3 network_to_shm.py --port 5005 --stream rgb_net
"""
import argparse
import socket
import struct
import time

from shm_bridge_python import shm_contract as C

WIRE = struct.Struct("<8I")        # magic, seq, frag, nfrags, w, h, ch|dtype, total_len
MAGIC = 0x53484D31
HDR_SZ = WIRE.size                  # 32 bytes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--stream", default="rgb_net")
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)   # big rx buffer
    sock.bind((args.bind, args.port))
    print(f"[network_to_shm] udp://{args.bind}:{args.port} -> /dev/shm/{args.stream}_*")

    hpath, fpath, rpath = C.shm_paths(args.stream)
    hmm = fmm = None
    cur_seq = None
    parts = {}            # frag_index -> bytes
    out_seq = 0

    while True:
        pkt, _ = sock.recvfrom(65535)
        if len(pkt) < HDR_SZ:
            continue
        magic, seq, frag, nfrags, w, h, ch_dt, total_len = WIRE.unpack_from(pkt, 0)
        if magic != MAGIC:
            continue
        channels = ch_dt & 0xFF
        dtype_id = (ch_dt >> 8) & 0xFF
        body = pkt[HDR_SZ:]

        if seq != cur_seq:               # new frame starts
            cur_seq, parts = seq, {}
        parts[frag] = body

        if len(parts) != nfrags:         # still waiting for fragments
            continue

        # all fragments present -> reassemble in order
        frame = b"".join(parts[i] for i in range(nfrags))
        if len(frame) != total_len:
            cur_seq = None               # corrupt/short; drop and resync
            continue

        # lazily create the local SHM buffers now that we know the size
        if hmm is None:
            _, hmm, _ = C.open_mmap(hpath, C.HEADER_SIZE, create=True)
            _, fmm, _ = C.open_mmap(fpath, max(total_len, 1), create=True)
            C.write_recipe(rpath, {
                "stream": args.stream, "encoding": "FLAT",
                "type_name": "sensor_msgs/msg/Image",
                "width": w, "height": h, "channels": channels,
                "dtype": C.DTYPE_BY_ID.get(dtype_id, "uint8"),
            })

        # publish under the seqlock (odd -> write -> even), same as write_py
        ts = time.monotonic_ns()
        out_seq += 1
        struct.pack_into("<I", hmm, 0, out_seq)
        fmm[0:total_len] = frame
        hmm[0:C.HEADER_SIZE] = C.pack_header(
            out_seq, C.ENC_FLAT, total_len, w, h, channels,
            dtype_id, ts, "sensor_msgs/msg/Image")
        out_seq += 1
        struct.pack_into("<I", hmm, 0, out_seq)
        cur_seq = None


if __name__ == "__main__":
    main()

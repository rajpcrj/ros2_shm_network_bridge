#!/usr/bin/env python3
"""
udp_client.py — subscribe to a shm_to_udp.cpp server and receive raw frames.

Sends "SUB" to the server, then reassembles the fragmented frames it streams back
(same 32-byte wire header as shm_to_network.cpp / network_to_shm.py). Prints one
line per fully-received frame. Sends "BYE" on exit.

Run (after the server is up on host:port):
    python3 udp_client.py --host 127.0.0.1 --port 6000
"""
import argparse
import socket
import struct
import time

WIRE = struct.Struct("<8I")          # magic, seq, frag, nfrags, w, h, ch|dtype, total_len
MAGIC = 0x53484D31
HDR = WIRE.size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6000)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    server = (args.host, args.port)
    sock.sendto(b"SUB", server)
    print(f"[udp_client] subscribed to udp://{args.host}:{args.port}  (Ctrl-C to stop)")

    cur_seq, parts = None, {}
    try:
        while True:
            pkt, _ = sock.recvfrom(65535)
            if len(pkt) < HDR:
                continue
            magic, seq, frag, nfrags, w, h, ch_dt, total = WIRE.unpack_from(pkt, 0)
            if magic != MAGIC:
                continue
            if seq != cur_seq:
                cur_seq, parts = seq, {}
            parts[frag] = pkt[HDR:]
            if len(parts) != nfrags:
                continue
            frame = b"".join(parts[i] for i in range(nfrags))
            if len(frame) != total:
                cur_seq = None
                continue
            channels = ch_dt & 0xFF
            print(f"seq={seq:>6}  {w}x{h}x{channels}  {len(frame)} bytes  "
                  f"first={frame[0] if frame else 0}")
            cur_seq = None
    except KeyboardInterrupt:
        pass
    finally:
        sock.sendto(b"BYE", server)
        print("\n[udp_client] unsubscribed.")


if __name__ == "__main__":
    main()

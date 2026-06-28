# Sending SHM data across the network (lowest-latency hop)

The bridge itself is **local only** — `/dev/shm` is not shared across machines.
To get a stream to another host, you read it locally and push the bytes over the
network. `examples/cpp/shm_to_network.cpp` (sender) + `examples/python/network_to_shm.py`
(receiver) implement this end to end.

## Why UDP is the lowest-latency choice here
For a one-way sensor/image stream, a plain **UDP datagram socket** is the
minimal-overhead network transport:

- **No discovery handshake** (DDS spends time/CPU on participant discovery).
- **No CDR (de)serialization** — we ship the raw frame bytes already sitting in
  shared memory.
- **No QoS state machine / ACK bookkeeping** on the hot path.

The cost is that UDP is **best-effort** (packets can drop/reorder). We add a compact
32-byte header with a per-frame `seq` and fragment indices so the receiver can
detect loss and reassemble. For a live video/sensor feed, dropping a stale frame is
usually preferable to head-of-line blocking — which is exactly UDP's behavior.

> **If you need guaranteed delivery**, switch the socket to TCP (`SOCK_STREAM`) —
> one line in `shm_to_network.cpp`. You trade a possible latency spike on loss
> (retransmit) for reliability. For multi-subscriber LAN fan-out, UDP multicast is
> another option (change the destination to a multicast group + `IP_ADD_MEMBERSHIP`
> on the receiver).

## Wire format (must match on both ends)
32-byte little-endian header per UDP packet, then payload:
```
uint32 magic      = 0x53484D31 ("SHM1")
uint32 seq        frame sequence number
uint32 frag       fragment index within this frame
uint32 nfrags     total fragments for this frame
uint32 width
uint32 height
uint32 channels   low 8 bits = channels, next 8 bits = dtype_id
uint32 total_len  total payload bytes for the whole frame
```
Frames larger than ~1400 B (safe sub-MTU payload) are split into `nfrags` packets
sharing the same `seq`; the receiver reassembles by `seq` and drops a frame if any
fragment is missing.

## Run it (two machines)
Machine A (has the camera / producer):
```bash
# 1) get the topic into /dev/shm
ros2 run shm_bridge_cpp ex_ros2_to_shm --ros-args -p topic:=/camera/image_raw -p stream:=rgb
# 2) push it to machine B
./shm_to_network --stream rgb --host 192.168.1.50 --port 5005
```
Machine B (192.168.1.50):
```bash
# receive UDP -> local /dev/shm/rgb_net_*
python3 examples/python/network_to_shm.py --port 5005 --stream rgb_net
# now any local reader sees it:
python3 examples/python/read_py.py rgb_net
# or expose it back onto B's ROS graph:
ros2 run shm_bridge_cpp ex_shm_to_ros2 --ros-args -p stream:=rgb_net -p topic:=/from_net
```

## Tuning for throughput
- Increase the receiver socket buffer (`SO_RCVBUF`, already 8 MiB in the example).
- Raise the NIC MTU to 9000 (jumbo frames) on a dedicated LAN and bump
  `MTU_PAYLOAD` to ~8900 to cut fragmentation.
- Pin the sender/receiver to isolated cores for jitter-sensitive streams.

## `shm_to_udp.cpp` — a UDP server that streams RAW shared memory
Where `shm_to_network.cpp` fires frames at ONE fixed host, `shm_to_udp.cpp` is a
real **UDP server**: it binds a port and streams the raw `/dev/shm` bytes to any
number of clients that subscribe. It is the lowest-overhead way to fan a shared-memory
stream onto the network — no ROS, no CDR, no transform; the bytes go straight from
`/dev/shm` onto the wire.

It combines, in one process:
1. a **Reader** on the `/dev/shm` stream (0 % CPU futex wait), and
2. a **UDP server** with a tiny control protocol.

Control protocol (client → server, one datagram):
- `"SUB"` — subscribe; the server starts streaming every new frame to you
- `"BYE"` — unsubscribe

Server → client packets use the **same 32-byte wire header** as above, so the same
receivers work. Run:
```bash
# server: serve /dev/shm/rgb_* on UDP :6000
./shm_to_udp --stream rgb --port 6000
# client: subscribe and receive raw frames
python3 examples/python/udp_client.py --host <server-ip> --port 6000
```
Multiple clients can subscribe at once; each gets the raw frames. (Verified
end-to-end: SHM writer → `shm_to_udp` server → Python `udp_client` reassembles
640×480×3 frames.)

## Caveat
This is a *minimal* transport meant to demonstrate the lowest-latency path. It does
not do congestion control, encryption, or retransmission. For production WAN use,
put it behind a real protocol (QUIC/SRT/RTP) or run ROS 2 over a tuned DDS — but on
a clean LAN, raw UDP from shared memory is about as low-latency as a one-way image
hop gets.

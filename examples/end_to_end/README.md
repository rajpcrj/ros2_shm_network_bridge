# end_to_end/ — ALL ROS 2 topics across the network, transparently

This is a complete **4-stage pipeline** that takes *every* ROS 2 topic on one
computer, ships it across a network, and reconstructs the *same* topics on another
computer — so a ROS node on the far side sees the data **as if nothing happened**.

![End-to-end: all ROS 2 topics from A reappear on B](../../docs/diagrams/pipeline.png)

Each stage is one small program. Data is **CDR** (the wire format ROS already uses),
so any message type works without compile-time knowledge. The topic name and full
type name travel inside the first UDP fragment of every frame (see `e2e_wire.hpp`), so
the receiver is fully self-describing — no shared config files between machines.

| stage | program | runs on | does |
|---|---|---|---|
| 1 | `e2e_1_ros2_to_shm` | machine A | every ROS topic → `/dev/shm` (+ a registry file) |
| 2 | `e2e_2_shm_to_udp` | machine A | `/dev/shm` streams → UDP datagrams to machine B |
| 3 | `e2e_3_udp_to_shm` | machine B | UDP → reassemble → `/dev/shm` (+ a registry) |
| 4 | `e2e_4_shm_to_ros2` | machine B | `/dev/shm` → re-publish the original ROS topics |

---

## Running it for real — TWO computers (the important part)

This pipeline is designed for **two different machines on the same network**. You
will almost certainly need to **`ssh` into the second machine** to launch stages 3
and 4 there, and you must make sure the two machines can actually reach each other.

### Before you start, on BOTH machines
1. **Network reachability.** Find each machine's IP (`ip addr` or `hostname -I`).
   From A, confirm you can reach B: `ping <B_IP>`.
2. **Open the UDP port.** Stage 2 sends to a UDP port (default 7000). On machine B,
   allow it through the firewall, e.g. `sudo ufw allow 7000/udp`.
3. **Same build on both.** Build this repo (or at least install the `.so`) on **both**
   machines — the `e2e_*` binaries must exist on each. Architectures can differ
   (rebuild per machine); the wire format is little-endian and arch-portable.
4. **(Optional) ssh access.** To drive everything from machine A, set up key-based
   ssh to B: `ssh-copy-id user@<B_IP>`, then you can launch B's stages remotely.

### Launch sequence

On **machine B** (destination) — e.g. over ssh from A:
```bash
ssh user@<B_IP>
source /opt/ros/humble/setup.bash && source <repo>/install/setup.bash
# stage 3: receive UDP -> /dev/shm
ros2 run shm_bridge_cpp e2e_3_udp_to_shm --port 7000 &
# stage 4: /dev/shm -> re-publish ROS topics
ros2 run shm_bridge_cpp e2e_4_shm_to_ros2 &
```

On **machine A** (source):
```bash
source /opt/ros/humble/setup.bash && source <repo>/install/setup.bash
# stage 1: all ROS topics -> /dev/shm
ros2 run shm_bridge_cpp e2e_1_ros2_to_shm &
# stage 2: /dev/shm -> UDP to machine B
ros2 run shm_bridge_cpp e2e_2_shm_to_udp --host <B_IP> --port 7000 &
```

Now on **machine B**, the topics from A appear in `ros2 topic list`, with their
original names and types. Verify: `ros2 topic echo <some_topic>`.

> **Why ssh?** The two halves run on two computers. There's no magic auto-deploy —
> you start stages 1–2 on A and stages 3–4 on B yourself, typically by ssh-ing into
> B (or sitting at each machine). A tmux/`ssh user@B 'cmd &'` workflow is common.

---

## Testing it RIGHT NOW on ONE computer (localhost)

You don't need two machines to sanity-check the network hop. Run the bundled test:

```bash
bash examples/end_to_end/test_localhost.sh
```

It fills a source `/dev/shm` stream with the sample image, runs **stage 2 → UDP
(127.0.0.1) → stage 3**, and reads the reassembled stream back to prove the bytes
arrived intact. Expected output:

```
PASS: stage 3 created the received stream (/dev/shm/got_*)
PASS: received seq=312 921600B type=sensor_msgs/msg/Image first=0
=== test_localhost done (rc=0) ===
```

(This is exactly how the pipeline was verified — the 921 600-byte image round-tripped
SHM → UDP → SHM byte-for-byte, with the topic/type metadata preserved.)

> **Why only the hop, not all 4 stages, on one machine?** Stage 1 and stage 3 both
> write the registry file `/dev/shm/e2e_streams.txt`, and on a single machine they'd
> share (and clobber) it. On two machines each has its own `/dev/shm`, so there's no
> conflict. The localhost test therefore isolates the network hop (stages 2↔3), which
> is the only part that actually crosses the wire.

---

## Caveats

- **UDP is best-effort.** Large frames are fragmented; if any fragment is lost, that
  frame is dropped (the next one still arrives). Fine for live sensor streams; for
  lossless transfer, switch the sockets to TCP (one change in stages 2/3) or run over
  a reliable tunnel.
- **No encryption / auth.** Anyone who can reach the UDP port receives the stream.
  For untrusted networks, tunnel it through ssh (`ssh -L`) or a VPN.
- **Bandwidth.** Shipping *all* topics (especially images) can saturate a link. In
  practice you'd filter to the topics you actually need (extend stage 1 with an
  allow-list) rather than bridging everything.
- **Clock/latency.** This pipeline preserves the message *bytes*, not timestamps in
  message headers — those are whatever the original publisher set.

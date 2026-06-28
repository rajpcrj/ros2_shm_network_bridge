# Architecture — how the SHM bridge works

## The idea in one paragraph
ROS 2's default transport (DDS) serializes every message to CDR bytes and delivers
a copy to each subscriber, spending CPU **per subscriber**. For large data (images,
point clouds) on a **single machine** that is wasteful. This bridge instead writes
each frame **once** into a POSIX shared-memory buffer in `/dev/shm`, and every
reader on that machine maps the same bytes. The **write** is O(1) and the
**RAM** is O(1) in reader count (one physical copy, mapped by all). The writer's
**publish/notify** is also O(1) — a single `FUTEX_WAKE` regardless of reader count.
What is *not* O(1) is total CPU: each reader still wakes and reads its own copy, so
delivering to N readers is **O(N) CPU** — inherently (you can't serve N consumers for
free). The bridge's win is therefore a **much lower per-subscriber CPU constant**
(~1.2 %/sub vs DDS ~10 %/sub), not a lower complexity class. DDS additionally pays
serialize + a kernel copy *per* subscriber, so its constant is far larger.

## The three files per stream
Each named stream `NAME` is three files in `/dev/shm`:

| file | purpose |
|---|---|
| `NAME_header` | fixed **64-byte** binary header (seqlock counter + shape + type) |
| `NAME_frame`  | the raw payload bytes (FLAT numeric buffer **or** CDR bytes) |
| `NAME_recipe.json` | human-readable decode recipe (topic, type, shape, dtype) |

The exact header layout is in [`shm_contract.hpp`](../src/shm_bridge_cpp/include/shm_bridge_cpp/shm_contract.hpp)
and mirrored byte-for-byte in [`shm_contract.py`](../src/shm_bridge_python/shm_bridge_python/shm_contract.py).
That shared layout is what makes C++ ↔ Python interop work with zero glue.

### Header (64 bytes, little-endian)
```
off  type        field
0    uint32      seq          <- the seqlock counter (even=stable, odd=writing)
4    uint32      encoding_id  <- 0=FLAT, 1=CDR
8    uint32      data_size    <- payload bytes in NAME_frame for THIS frame
12   uint32      width
16   uint32      height
20   uint32      channels
24   uint32      dtype_id     <- 0=u8 1=i8 2=u16 3=i16 4=u32 5=i32 6=f32 7=f64
28   uint32      reserved
32   uint64      timestamp_ns
40   char[24]    type_name    <- e.g. "sensor_msgs/msg/Image" (null-padded)
```

## The seqlock (lock-free, writer never blocks)
The writer never waits on readers. Protocol:

1. `seq++`  → now **odd** = "a write is in progress"
2. copy the payload into `NAME_frame`, fill the header fields
3. `seq++`  → now **even** = "stable / published"

A reader does **read seq → read body → re-read seq**; the frame is valid only if
both reads saw the **same even** value (otherwise it caught a write in progress and
retries). This is the classic seqlock; it favors the writer and is correct for a
single writer + many readers. See [`02_data_flow.md`](02_data_flow.md).

## Wakeup: futex, not busy-poll (one O(1) wake for all readers)
A naive reader would spin-read the seq counter, burning a core per reader. Instead
the header's seq word doubles as a **futex**. After publishing, the writer calls
`notify()` → one `FUTEX_WAKE` releases **all** waiting readers at once, regardless
of how many there are (the writer's notify is O(1)). Readers call
`Reader::wait_and_read()` which `FUTEX_WAIT`s at ~0% CPU until woken. Total delivery
CPU still grows with readers — each reader wakes and reads its own copy, so it's
**O(N)** — but with a low slope: measured ~1.2 %/sub vs DDS ~10 %/sub (see
[../test_runs/README.md](../test_runs/README.md)). The win is a small constant, not a
lower complexity class.

Implementation: [`shm_futex.hpp`](../src/shm_bridge_cpp/include/shm_bridge_cpp/shm_futex.hpp).

## FLAT vs CDR encoding
- **FLAT** (`encoding_id=0`): the raw numeric buffer (e.g. image pixels) copied
  verbatim, with `width/height/channels/dtype` describing the shape. Readers
  reconstruct without any deserialization. Best for big array-like messages.
- **CDR** (`encoding_id=1`): the message serialized to DDS CDR bytes. Type-agnostic
  (works for any ROS message) but the reader must deserialize to get fields. Used
  by the generic writer for arbitrary types.

## What lives where
```
src/shm_bridge_cpp/      the C++ library (libshm_bridge_cpp.so) + nodes + benchmarks
src/shm_bridge_python/   the Python package (same contract, ROS nodes, helpers)
examples/cpp/            copy-paste C++ programs (this set)
examples/python/         copy-paste Python programs (this set)
docs/                    these guides
install_lib.sh           copy .so + headers to /usr/local for system-wide use
```

## When to use this (and when not to)
**Use it** for high-rate, large-payload, **same-machine** fan-out to many readers,
where CPU and copies matter (multi-consumer perception, recording + viz + inference
off one camera). **Don't** use it as a network transport by itself — it is local
only. For the cross-machine hop, see [`04_network_transport.md`](04_network_transport.md)
(`shm_to_network.cpp`). For single-subscriber minimum latency, plain DDS data-sharing
(loaned messages) is competitive or better — this bridge's win is CPU and scale,
not single-reader latency.

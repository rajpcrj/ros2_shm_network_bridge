# Intra-process vs inter-process benchmarking — in detail

This is the single most important methodology choice in any ROS 2 / middleware
benchmark, and the one our current results depend on most. This document explains
what each mode is, the mechanics of *why* they differ, what costs appear (or vanish)
in each, which one our harness used, and how to read our numbers because of it.

---

## 1. The two words, precisely

A "process" is an OS-level unit with its **own virtual address space**, its own file
descriptors, and its own scheduling entity. Threads share a process (and its memory);
separate processes do not.

- **Intra-process benchmark** — the publisher and all subscribers are **threads in
  ONE process**. They share one address space. A pointer in the publisher is valid in
  the subscriber. (This is what our `test4_benchmarks` harness does: one process,
  `std::thread` per subscriber.)

- **Inter-process benchmark** — the publisher and each subscriber are **separate
  processes** (separate `ros2 run …` invocations / separate binaries). They do **not**
  share an address space. Moving data between them requires the OS (shared memory
  segments, sockets, pipes) and crossing a process boundary.

ROS 2 itself supports both: nodes can be **composed** into one process
(intra-process communication, IPC) or run as **separate nodes** (the normal
deployment). `performance_test` likewise has an intra-process mode and an
inter-process mode, and the **headline DDS numbers people cite are usually
inter-process**, because that is how robots actually run.

---

## 2. Why they differ — the mechanics

The difference is not cosmetic. Different *physical work* happens in each mode.

### (a) Data movement
- **Intra-process:** the data is already in the shared heap. The fastest transports
  can hand over a **pointer** — zero copies. Even a "copy" is a single in-RAM memcpy
  within one address space.
- **Inter-process:** the bytes must leave one address space and enter another. The
  realistic mechanisms:
  - **Shared memory** (`/dev/shm`, DDS data-sharing, iceoryx): one copy into a shared
    segment, then readers map it. (This is the bridge's whole premise — and it's the
    case where the bridge and a shared-memory DDS look most similar.)
  - **Loopback sockets / DDS over UDP:** the kernel copies the bytes user→kernel→user
    for every subscriber. Expensive for large payloads.

### (b) Discovery & connection setup
- **Intra-process:** none. The threads just share variables; "connecting" a
  subscriber is adding it to a list.
- **Inter-process:** DDS participants must **discover each other** over the network
  (multicast/unicast handshakes, endpoint matching, QoS negotiation). This costs CPU
  and time at startup and ongoing liveliness traffic. A shared-memory DDS still does
  full RTPS discovery even though the data path is shared memory. **Our intra-process
  harness skipped all of this.**

### (c) Scheduling & wakeups
- **Intra-process:** waking a subscriber is a thread wakeup (a futex/condvar within
  the process). Cheap, and the scheduler sees one process.
- **Inter-process:** waking a subscriber in another process is a **cross-process**
  wakeup (futex on a shared segment, or a socket becoming readable). The OS scheduler
  juggles N+1 separate processes, each possibly with its own executor thread pool,
  all competing for cores. Context-switch and cache-locality costs rise with the
  number of processes.

### (d) CPU accounting
- **Intra-process:** all the work shows up in **one** process's CPU. Easy to measure
  with `/proc/self/stat`.
- **Inter-process:** the work is spread across N+1 processes **plus daemons** (e.g.
  `iox-roudi`). A naive per-process CPU read **under-counts** the transport. (This is
  exactly why our harness uses a **pid-set CPU** that sums the benchmark process + the
  iox-roudi daemon — even in intra-process mode it tries to be daemon-fair, but in
  true inter-process mode the per-subscriber-process CPU would also need summing.)

### (e) Memory
- **Intra-process:** one copy of the data in the heap, shared by all reader threads.
- **Inter-process:** with shared memory, still ~one copy in the segment; with
  socket-based transport, **one copy per subscriber** crosses the kernel — O(n) RAM
  traffic.

---

## 3. Who is favored by each mode

This is the crux of why the choice matters for *our* conclusions.

- **Intra-process FAVORS / FLATTERS transports in two opposite ways at once:**
  - It **removes costs DDS pays in production** (discovery, multi-process scheduling,
    cross-process wakeups). So DDS can look **cheaper and faster than it really is
    when deployed.** → this *understates the bridge's real-world advantage.*
  - It also makes "in-process shared memory" (some DDS data-sharing paths) and our
    `/dev/shm` path look more alike than they are at deployment, because neither pays
    the cross-process tax in this mode.

- **Inter-process is the realistic, deployment-accurate test.** It's what a robot
  actually does (camera node, perception node, logger node = separate processes). It
  exposes discovery cost, per-process executor cost, and the true CPU spread.

**Net:** our intra-process numbers are a **clean, controlled lower bound on overhead
for every transport.** They correctly rank the transports and correctly show the CPU
scaling slope (the futex win is real in both modes). But the *magnitude* of the
bridge's advantage over DDS is very likely **larger** in the realistic inter-process
case, because DDS pays more there and the bridge pays roughly the same.

---

## 4. What our harness actually used, and why

We used **intra-process** (1 process, 1 publisher thread + N subscriber threads).
Reasons:

1. **It isolates the transport mechanism.** With everything in one process, the only
   variable is the publish/subscribe path; we removed inter-process scheduling noise
   so the latency/CPU differences are attributable to the transport, not the OS.
2. **One-way latency is valid in-process.** Pub and sub share one `CLOCK_MONOTONIC`
   epoch, so a direct send−receive subtraction is meaningful without a round-trip or
   clock sync (see `ROUND_TRIP.md`).
3. **Simplicity and auditability.** One small harness, one stats path, byte-identical
   payloads across transports.

The cost of that choice is everything in §3: we did **not** measure the realistic
inter-process deployment, and our absolute numbers are therefore **not comparable to
published inter-process `performance_test` results.**

---

## 5. How to read our results because of this

- ✅ **Trust the relative ranking and the CPU *slope*.** Bridge ≈ 1.2 %/sub vs DDS
  ≈ 10 %/sub is a structural property of the futex (no busy-poll) that holds in both
  modes. The crossover and the CycloneDDS crash are real.
- ⚠️ **Do not trust absolute milliseconds as deployment numbers.** Real
  (inter-process) latency will be higher for all transports, and DDS's CPU will be
  higher still (discovery + executors).
- ⚠️ **Expect the bridge's advantage to GROW, not shrink, inter-process.** The work
  we removed is mostly DDS's; the bridge's shared-memory + futex path changes least
  when you split processes.

---

## 6. What the inter-process variant will add

To close this gap we will run each transport with pub and each subscriber as
**separate processes**:
- DDS will pay real **participant discovery** and run **N+1 executors** competing for
  cores → its CPU and tail latency should rise.
- The bridge's `Writer`/`Reader` already work across processes unchanged (that's the
  whole point of `/dev/shm`); only the harness wiring (spawn processes instead of
  threads) changes.
- Latency must then be measured **round-trip** (no shared clock across processes) —
  see `ROUND_TRIP.md`.
- CPU must sum **all** participating processes' pids + daemons (the pid-set approach
  generalizes directly).

The result will be a second, deployment-accurate column beside the current
intra-process one — and, we expect, a *wider* bridge advantage on CPU/scale.

---

## 7. One-line summary

> **Intra-process** = pub + subs as threads in one address space: cheapest, removes
> DDS's deployment costs, valid for one-way latency, what we ran — a clean *relative*
> comparison. **Inter-process** = separate processes: realistic deployment, exposes
> discovery + multi-executor + cross-process costs, needs round-trip latency, not yet
> run — the number you'd quote to the outside world. They differ because real work
> (discovery, copies across address spaces, multi-process scheduling) exists in one
> and not the other.

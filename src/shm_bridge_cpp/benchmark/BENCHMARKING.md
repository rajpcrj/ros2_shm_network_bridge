# How this was benchmarked — and how it differs from the standard

This folder contains a **custom** benchmark harness, not the upstream Apex.AI
`performance_test`. This document explains exactly **how** it measures, **why** we
did not just run the standard tool, where our method is **better**, where it is
**worse**, and **what we deliberately skipped**. Read this before quoting any
number — especially before comparing our absolute numbers to published
`performance_test` results.

---

## 1. What the harness actually does

Files:
- `bench_common.hpp` — shared timing/stats/CPU/gate code (the "frozen" core).
- `test4_benchmarks/` — the comparison used for the published graphs: `bridge_bench4`,
  `fastdds_bench4`, `cyclonedds_bench4`, `shm_msgs_bench4`, plus `bench_cpu4.hpp`
  (CPU accounting) and `bench4_report.hpp` (aggregation/CSV).

Measurement model:
- **One process, one publisher thread + N subscriber threads.** Pub and sub live in
  the *same* process.
- **Latency is ONE-WAY, via a shared in-process clock.** The publisher writes
  `mono_ns()` (a `CLOCK_MONOTONIC` reading) into the first 8 bytes of the payload;
  each subscriber computes `mono_ns() - sent` on receipt. Because both threads are
  in the same process they share the same monotonic epoch, so the subtraction is a
  valid one-way latency.
- **Fixed rate, absolute schedule.** The publisher fires at a fixed Hz using
  `sleep_until(t0 + k*period)` — **not** `sleep_for(period)`. This avoids
  **coordinated omission**: if the system stalls, the next frame still gets its true
  intended timestamp, so a stall shows up as high latency instead of vanishing.
- **The writer never blocks on readers.** It publishes on schedule regardless of
  whether readers keep up; dropped frames are detected via gaps in the sequence
  number, not hidden.
- **Percentiles + warm-up trim.** Per run we pool all subscribers' samples, discard
  the first 20 % (cache/JIT/page-fault settle), and report p50/p90/p99/p99.9/max.
- **K runs → mean ± std-dev.** Each point is run K times; we report the spread
  across runs (the error bars in the graphs).
- **Whole-system-fair CPU (pid-set).** Headline CPU sums the benchmark process **+
  named daemons** (e.g. `iox-roudi` for CycloneDDS) so DDS gets no free ride for
  work it offloads to a daemon, and the figure is immune to unrelated desktop load.
  Raw whole-system CPU and a pre-run idle floor are recorded as disclosed secondary
  columns.
- **Health gates.** System CPU/RAM > 95 %, a lost subscriber, a crash, or a stall
  are all logged (`stops.log`) and stop only the affected transport.
- **Measurement overhead disclosed.** `measure_overhead_ns()` times the timestamp
  read/write loop so we know the noise floor of the instrument itself.

---

## 2. Why we did NOT just run the standard `performance_test`

`performance_test` is excellent and is the reference tool. We built a custom harness
for specific reasons, each a deliberate trade-off:

1. **Our transport is not a ROS middleware (RMW).** The `/dev/shm` bridge is a
   plain library (`libshm_bridge_cpp.so`) with its own `Writer`/`Reader` API — it is
   **not** a pluggable `rmw_*` implementation. `performance_test` drives transports
   through a common plugin abstraction (its `communicator` interface). To put our
   bridge into `performance_test` we'd have to write a full RMW shim or a custom
   communicator plugin — a large effort that would also wrap our API in
   `performance_test`'s assumptions (typed messages, its loaning model, etc.),
   muddying exactly the thing we wanted to measure (the raw library).

2. **We needed to measure things `performance_test` doesn't isolate.** Specifically:
   the **deserialization asymmetry** (zero-copy vs deserialize, both sides), the
   **pid-set CPU** that captures the iox-roudi daemon, and a **bridge "deserialize"
   mode** that pays the same per-frame copy DDS pays. These required custom
   instrumentation in the receive path.

3. **Single, controlled variable.** We wanted byte-identical payloads, identical
   rate, identical schedule, identical stats code across all four transports, with
   **only the publish/subscribe mechanism swapped**. Writing one small harness made
   that guarantee trivial to audit (it's ~one `bench_common.hpp`).

4. **We borrowed `performance_test`'s *methodology*, not its code.** The conventions
   above (fixed-rate absolute schedule, percentiles, warm-up, multiple runs,
   coordinated-omission avoidance, fixed-size message ladder) are exactly what
   `performance_test` established. We reimplemented those conventions; we did not
   reinvent them.

---

## 3. Where our method is BETTER (for our goal)

- **Perfectly isolated variable.** Identical timing/stats/payload code; only the
  transport differs. Hard to achieve when each transport is a different RMW under
  `performance_test`.
- **Symmetric zero-copy vs deserialize on BOTH sides.** We added a bridge
  *deserialize* mode and DDS *loaned* mode so neither side is unfairly credited for
  skipping work. `performance_test`'s default does not frame it as this explicit
  2×2.
- **Daemon-fair CPU.** Our pid-set CPU counts `iox-roudi`. A naive per-process CPU
  read (what you'd get from many simple harnesses) would under-count CycloneDDS.
- **Failure logging.** Crashes/gates/stalls are first-class outputs, not run
  failures to be retried away. The CycloneDDS N=32 crash and the 8 MiB segfault are
  *results*, not omissions.

---

## 4. Where our method is WORSE / less standard

These are the important caveats. **Do not ignore them.**

- **Intra-process (threads), not inter-process.** Pub and sub are threads in ONE
  process. Real ROS 2 deployments run nodes in **separate processes**, where DDS
  additionally pays **participant discovery**, **separate executors competing for
  cores**, and **cross-process wakeups**. By staying in-process we **removed costs
  the DDS transports pay in production**. This can *understate* the bridge's
  real-world advantage (DDS looks cheaper than it is when deployed) — or, for a
  shared-memory DDS path, blur the line between "in-process shared memory" and our
  `/dev/shm`. Either way, **the single-process result is not the deployment
  result.** `performance_test` supports both modes; the headline DDS numbers people
  cite are usually **inter-process**. We only ran intra-process here.
  → *An inter-process variant is the planned follow-up; see §6.*

- **One-way latency, not round-trip.** We measure publisher→subscriber one-way using
  a shared clock. `performance_test` typically measures a **round trip** (a relay
  echoes the message back) and reports **round-trip/2**, because that needs only ONE
  clock and survives pub/sub being on different machines/processes with unsynced
  clocks. Consequences:
  - Our one-way number is **not** "round-trip ÷ 2" and should not be compared to a
    round-trip figure.
  - Our one-way is only valid **because** pub and sub share a process/clock — it
    would be *invalid* across machines without clock sync. (This is precisely why
    `performance_test` uses round-trip.)
  → *A round-trip latency path matching the perf_test convention is also planned;
    see §6, and the detailed explanation of how perf_test does it is in
    `ROUND_TRIP.md`.*

- **Absolute numbers are NOT comparable to published `performance_test` results.**
  Because of the two points above (in-process + one-way + `powersave` governor), our
  absolute milliseconds are **lower** than a standard inter-process round-trip run
  would report. **Our numbers are only valid for comparing the four transports to
  EACH OTHER under identical conditions** — not against any external paper/number.

- **Governor was `powersave`, turbo on, no core isolation, a desktop in the
  background.** `performance_test` rigs usually pin `performance` governor, isolate
  CPUs (`isolcpus`), and disable turbo for repeatability. We could not set the
  governor (no sudo). This inflates absolute latency **equally** for all transports
  (so relative comparison holds) but adds run-to-run jitter.

- **Single machine, single NIC config, single OS.** No cross-distro / cross-arch /
  cross-kernel sweep.

---

## 5. What we COMPLETELY SKIPPED from the standard benchmark

Things `performance_test` (or a rigorous transport study) does that we did **not**:

- **Inter-process / distributed runs** — skipped (intra-process only). *Planned.*
- **Round-trip latency + relay node** — skipped (one-way only). *Planned.*
- **Multiple QoS profiles** — we used a single `KeepLast(1)` / SensorData-style QoS.
  No reliability-vs-best-effort sweep, no history-depth sweep, no deadline/liveliness
  QoS testing.
- **Multiple publishers / many-to-many** — we tested **1 publisher → N subscribers**
  only. No N-publisher fan-in, no full mesh.
- **Message-type variety** — we used flat image-like `uint8` buffers (the FLAT path)
  and the fixed `Image*` types. We did **not** sweep `performance_test`'s
  Array/Struct/PointCloud/nested-type ladders, nor variable-length string/sequence
  types, nor the CDR path of our own bridge.
- **Small-payload rungs** — we dropped 8 KiB / 512 KiB; only 1–8 MiB. So we say
  nothing rigorous about small-message latency (where DDS is often strongest).
- **Sustained / soak testing** — runs are 10 s. No hour-long stability, no
  memory-leak / fragmentation watch, no thermal-throttle-over-time check.
- **Loss/recovery under network conditions** — N/A (local only); no packet-loss,
  jitter, or reordering injection.
- **CPU pinning / NUMA placement** — no affinity sweep beyond the optional core pin;
  no NUMA-aware vs naive comparison.
- **Cross-clock / clock-skew handling** — not needed (shared clock), but it means
  the harness can't be used across machines as-is.
- **Statistical rigor beyond K=4** — K=4 gives a std-dev but not tight confidence
  intervals; no outlier analysis, no distribution fitting.

---

## 6. Planned additions (to close the gap with the standard)

1. **Inter-process variant** — run publisher and each subscriber as separate
   processes so DDS pays realistic discovery + executor costs. This is the
   deployment-accurate check.
2. **Round-trip latency path** — add a relay so latency is measured as round-trip/2,
   matching `performance_test` and enabling cross-process/cross-machine timing
   without a shared clock. The detailed walkthrough of how `performance_test`'s main
   loop implements this is in `ROUND_TRIP.md`.

Until those land, treat every number here as: **"a fair RELATIVE comparison of four
local transports under identical in-process, one-way conditions"** — nothing more,
nothing less.

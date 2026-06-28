// shm_futex.hpp — Linux futex wake/wait on the seqlock word (header offset 0).
//
// Why: the seqlock reader otherwise has to busy-poll to notice a new frame, which
// costs ~1 core per reader and is the only O(n)-in-subscribers CPU cost in the
// bridge (the write/fan-out itself is O(1)). A futex turns that into:
//   - writer: ONE FUTEX_WAKE(addr, INT_MAX) after each frame  -> O(1), wakes all
//             waiting readers regardless of how many there are;
//   - reader: FUTEX_WAIT(addr, last_seq) sleeps at ~0% CPU until the seq changes.
//
// The futex address is the 32-bit seqlock counter at header offset 0 — the same
// word the seqlock already bumps every publish. No new shared-memory fields, and
// it stays byte-compatible with readers/writers that don't use the futex (they
// just keep polling). A missed wake is harmless: a poller still sees the new seq,
// and a futex waiter re-checks the value before sleeping (no lost-wakeup race).
#pragma once

#include <cstdint>
#include <ctime>

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace shm_futex {

inline int futex(uint32_t* uaddr, int op, uint32_t val,
                 const struct timespec* to = nullptr) {
    return static_cast<int>(syscall(SYS_futex, uaddr, op, val, to, nullptr, 0));
}

// Wake up to `count` waiters on the seqlock word (use INT32_MAX to wake all).
// Private futex: faster, fine because all readers mmap the SAME file page, so the
// kernel keys on the underlying physical page — shared across processes.
inline void wake(void* hdr_mem, int count = 0x7fffffff) {
    auto* w = reinterpret_cast<uint32_t*>(hdr_mem);
    futex(w, FUTEX_WAKE, static_cast<uint32_t>(count));
}

// Block until the seqlock word differs from `expected`, or until `timeout_ns`
// elapses (0 = no timeout). Returns the current seq value (which may still equal
// `expected` on spurious wake / timeout — caller re-checks). The kernel only
// sleeps if *addr still == expected at the moment of the call, so there is no
// lost-wakeup window vs the writer's wake.
inline uint32_t wait(void* hdr_mem, uint32_t expected, uint64_t timeout_ns = 0) {
    auto* w = reinterpret_cast<uint32_t*>(hdr_mem);
    if (timeout_ns == 0) {
        futex(w, FUTEX_WAIT, expected, nullptr);
    } else {
        struct timespec to;
        to.tv_sec = static_cast<time_t>(timeout_ns / 1000000000ULL);
        to.tv_nsec = static_cast<long>(timeout_ns % 1000000000ULL);
        futex(w, FUTEX_WAIT, expected, &to);
    }
    return __atomic_load_n(w, __ATOMIC_ACQUIRE);
}

}  // namespace shm_futex

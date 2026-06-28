#!/usr/bin/env python3
"""
make_graphs.py — render the benchmark comparison graphs from data/sweep.csv into
graphs/*.png. Pure stdlib + matplotlib (no pandas). Re-run after a new benchmark:
    python3 make_graphs.py
"""
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "data", "sweep.csv")
OUT = os.path.join(HERE, "graphs")
os.makedirs(OUT, exist_ok=True)

# friendly label + plot style per (transport, mode)
SERIES = {
    ("bridge_shm_futex", "raw_flat"):      ("bridge (raw, zero-deserialize)", "o-",  "#1b9e77"),
    ("bridge_shm_futex", "deserialize"):   ("bridge (deserialize)",            "o--", "#66c2a5"),
    ("fastdds_datasharing", "loaned"):     ("FastDDS (loaned/zero-copy)",      "s-",  "#d95f02"),
    ("fastdds_datasharing", "normal"):     ("FastDDS (normal/deserialize)",    "s--", "#fc8d62"),
    ("cyclonedds_iceoryx", "loaned"):      ("CycloneDDS (loaned)",             "^-",  "#7570b3"),
    ("cyclonedds_iceoryx", "normal"):      ("CycloneDDS (normal)",             "^--", "#8da0cb"),
    ("ros2_shm_msgs_Image1m", "loaned"):   ("ros2_shm_msgs (loaned)",          "D-",  "#e7298a"),
    ("ros2_shm_msgs_Image1m", "normal"):   ("ros2_shm_msgs (normal)",          "D--", "#e78ac3"),
}


def load():
    rows = []
    with open(CSV) as f:
        for r in csv.DictReader(f):
            try:
                r["subs"] = int(r["subs"]); r["bytes"] = int(r["bytes"])
                for k in ("p50_ms", "p99_ms", "cpu_pidset_pct", "fps_per_sub",
                          "lost_pct", "p50_sd", "cpu_pidset_sd"):
                    r[k] = float(r.get(k, 0) or 0)
                rows.append(r)
            except (ValueError, KeyError):
                pass
    return rows


def series_key(r):
    # ros2_shm_msgs transport name carries the size; normalize to one key
    tp = r["transport"]
    if tp.startswith("ros2_shm_msgs"):
        tp = "ros2_shm_msgs_Image1m"
    return (tp, r["mode"])


def plot_vs_subs(rows, ycol, ysd, title, ylabel, fname, size_bytes, logy=False):
    plt.figure(figsize=(9, 5.5))
    for key, (label, style, color) in SERIES.items():
        pts = sorted([r for r in rows
                      if series_key(r) == key and r["bytes"] == size_bytes],
                     key=lambda r: r["subs"])
        if not pts:
            continue
        xs = [p["subs"] for p in pts]
        ys = [p[ycol] for p in pts]
        es = [p[ysd] for p in pts]
        plt.errorbar(xs, ys, yerr=es, fmt=style, color=color, label=label,
                     capsize=3, markersize=6, linewidth=1.8)
    plt.xlabel("subscribers (N)")
    plt.ylabel(ylabel)
    if logy:
        plt.yscale("log")
    plt.title(f"{title}  —  {size_bytes // (1024*1024)} MiB frames, K=4")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8, ncol=2)
    plt.tight_layout()
    p = os.path.join(OUT, fname)
    plt.savefig(p, dpi=120)
    plt.close()
    print("wrote", p)


def plot_vs_size(rows, ycol, title, ylabel, fname, subs=1):
    plt.figure(figsize=(9, 5.5))
    for key, (label, style, color) in SERIES.items():
        pts = sorted([r for r in rows
                      if series_key(r) == key and r["subs"] == subs],
                     key=lambda r: r["bytes"])
        if not pts:
            continue
        xs = [p["bytes"] // (1024 * 1024) for p in pts]
        ys = [p[ycol] for p in pts]
        plt.plot(xs, ys, style, color=color, label=label, markersize=6, linewidth=1.8)
    plt.xlabel("payload size (MiB)")
    plt.ylabel(ylabel)
    plt.title(f"{title}  —  N={subs} subscriber, K=4")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8, ncol=2)
    plt.tight_layout()
    p = os.path.join(OUT, fname)
    plt.savefig(p, dpi=120)
    plt.close()
    print("wrote", p)


def main():
    rows = load()
    MiB = 1024 * 1024
    # 1) CPU vs subscribers @1MiB — the headline O(1) vs O(n) story
    plot_vs_subs(rows, "cpu_pidset_pct", "cpu_pidset_sd",
                 "CPU per core vs subscribers", "CPU (% of one core)",
                 "cpu_vs_subs_1m.png", 1 * MiB)
    # 2) latency p50 vs subscribers @1MiB — the crossover
    plot_vs_subs(rows, "p50_ms", "p50_sd",
                 "Latency (p50) vs subscribers", "p50 latency (ms)",
                 "latency_vs_subs_1m.png", 1 * MiB)
    # 3) delivered fps vs subscribers @1MiB — integrity
    plot_vs_subs(rows, "fps_per_sub", "lost_pct",
                 "Delivered rate vs subscribers (target 30)", "fps per subscriber",
                 "fps_vs_subs_1m.png", 1 * MiB)
    # 4) latency vs payload size @N=1 — O(1) zero-copy vs O(size) copy
    plot_vs_size(rows, "p50_ms",
                 "Latency (p50) vs payload size", "p50 latency (ms)",
                 "latency_vs_size_n1.png", subs=1)
    # 5) CPU vs subscribers @4MiB — same story holds at a bigger size
    plot_vs_subs(rows, "cpu_pidset_pct", "cpu_pidset_sd",
                 "CPU per core vs subscribers", "CPU (% of one core)",
                 "cpu_vs_subs_4m.png", 4 * MiB)


if __name__ == "__main__":
    main()

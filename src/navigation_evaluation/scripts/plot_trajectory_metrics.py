#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main() -> int:
    csv_path = Path(sys.argv[1] if len(sys.argv) > 1 else "evaluations/results/trajectory_metrics.csv")
    out_dir = Path(sys.argv[2] if len(sys.argv) > 2 else "evaluations/plots")
    if not csv_path.is_file():
        print(f"Missing {csv_path}", file=sys.stderr)
        return 1

    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    if not rows:
        print("CSV empty", file=sys.stderr)
        return 1

    out_dir.mkdir(parents=True, exist_ok=True)
    labels = [f"{r['fusion_mode']}/{r['lidar_mode']}" for r in rows]
    ate = [float(r["ate_xy_m"]) for r in rows if r["ate_xy_m"]]
    if len(ate) != len(rows):
        print("Skipping plot: missing numeric metrics", file=sys.stderr)
        return 2

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.bar(range(len(labels)), ate)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("ATE xy [m]")
    ax.set_title("Odometry vs ground truth")
    fig.tight_layout()
    fig.savefig(out_dir / "ate_xy_comparison.png", dpi=120)
    plt.close(fig)
    print(f"Wrote {out_dir / 'ate_xy_comparison.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

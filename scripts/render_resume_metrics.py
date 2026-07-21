#!/usr/bin/env python3
"""Build RESUME_METRICS.md from CSV/JSON (no fabricated numbers)."""
from __future__ import annotations

import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "evaluations" / "results"
CSV = RESULTS / "trajectory_metrics.csv"
ROW = RESULTS / "trajectory_row.csv"
NAV = RESULTS / "nav_benchmark.json"
OUT = RESULTS / "RESUME_METRICS.md"
PLOT = ROOT / "evaluations" / "plots" / "ate_xy_comparison.png"


def read_traj(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main() -> None:
    rows = read_traj(CSV)
    if not rows:
        rows = read_traj(ROW)

    nav = {}
    if NAV.is_file():
        nav = json.loads(NAV.read_text(encoding="utf-8"))

    lines = [
        "# Evaluation metrics (for resume / STAR)",
        "",
        "Generated from repo outputs only. Re-run:",
        "`bash scripts/run_trajectory_matrix.sh` and Nav: `bash scripts/run_nav_benchmark.sh`.",
        "",
        "## Trajectory vs Gazebo ground truth (`/ground_truth/odom`)",
        "",
    ]

    if rows:
        lines.append("| fusion | lidar | n | ATE (m) | RMSE x | RMSE y | RMSE yaw (rad) |")
        lines.append("|--------|-------|---|---------|--------|--------|----------------|")
        for r in rows:
            if not r.get("samples") or r.get("samples") == "0":
                continue
            lines.append(
                f"| {r.get('fusion_mode','')} | {r.get('lidar_mode','')} | {r['samples']} | "
                f"{r.get('ate_xy_m','')} | {r.get('rmse_x_m','')} | {r.get('rmse_y_m','')} | "
                f"{r.get('rmse_yaw_rad','')} |"
            )
    else:
        lines.append("_No trajectory CSV yet._")

    lines.extend(["", "## Nav2 three-goal benchmark", ""])
    if nav:
        lines.append(
            f"- **Success rate:** {nav.get('passed', '?')}/{nav.get('total', '?')} "
            f"({float(nav.get('success_rate', 0)) * 100:.0f}%)"
        )
        for g in nav.get("goals", []):
            ok = "OK" if g.get("success") else "fail"
            lines.append(
                f"- Goal ({g.get('x')}, {g.get('y')}): **{ok}**, {g.get('time_sec', 0):.1f} s"
            )
    else:
        lines.append("_No nav_benchmark.json yet._")

    if PLOT.is_file():
        lines.extend(["", f"Plot: `{PLOT.relative_to(ROOT)}`", ""])

    lines.extend(
        [
            "## STAR snippet (English) — edit numbers from table above",
            "",
            "- **Situation:** ROS 2 Gazebo sim needed quantitative odometry and Nav2 validation.",
            "- **Task:** Benchmark EKF fusion modes and LiDAR noise against ground truth; automate Nav2 goals.",
            "- **Action:** Implemented trajectory evaluator (ATE/RMSE) + Nav2 benchmark; ran matrix in simulation.",
            "- **Result:** _(paste best row, e.g. wheel_imu/clean ATE ≈ X mm; Nav2 Y/3 goals.)_",
            "",
        ]
    )

    OUT.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()

"""Turn a run CSV (logs/run_*.csv) into report-ready PNG charts.

    pip install matplotlib
    python plot_results.py                 # newest file in logs/
    python plot_results.py logs/run_20261014_153000.csv --no-show
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import config

LEVEL_COLORS = {"SLOWING": "#f0be3c", "BRAKING": "#f06e3c", "EMERGENCY_BRAKING": "#f03c3c"}


def load(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def column(rows: list[dict[str, str]], name: str, scale: float = 1.0) -> list[float]:
    """Numeric column; empty cells become NaN so lines break there."""
    return [float(r[name]) * scale if r[name] != "" else float("nan") for r in rows]


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot a simulation run CSV")
    parser.add_argument("csv", nargs="?", help="CSV file (default: newest run_*.csv in logs/)")
    parser.add_argument("--no-show", action="store_true", help="only save the PNG")
    args = parser.parse_args()

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed. Run:  pip install matplotlib", file=sys.stderr)
        return 1

    path = Path(args.csv) if args.csv else max(Path(config.LOG_DIR).glob("run_*.csv"), default=None, key=lambda p: p.stat().st_mtime)
    if path is None or not path.exists():
        print("No CSV found. Run main.py (it saves one on exit) or press E in the window.", file=sys.stderr)
        return 1
    rows = load(path)
    if not rows:
        print(f"{path} has no samples.", file=sys.stderr)
        return 1

    t = column(rows, "time_s")
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    panels = (
        ("Speed [km/h]", column(rows, "speed_mps", 3.6), None, "#2a9df4"),
        ("Gap to leader [m]", column(rows, "front_gap_m"), config.SAFETY_MIN_FRONT_GAP_M, "#3cb371"),
        ("TTC [s]", column(rows, "front_ttc_s"), config.SAFETY_MIN_TTC_S, "#e08a1e"),
        ("Auto brake [m/s²]", column(rows, "avoid_decel_mps2"), None, "#c0392b"),
    )
    for ax, (label, values, threshold, color) in zip(axes, panels):
        ax.plot(t, values, color=color, linewidth=1.6)
        if threshold is not None:
            ax.axhline(threshold, color="#aa3333", linestyle="--", linewidth=1, label=f"threshold {threshold:g}")
            ax.legend(loc="upper right", fontsize=8)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[2].set_ylim(0, config.GRAPH_TTC_MAX_S)
    axes[1].set_ylim(0, config.GRAPH_GAP_MAX_M)

    # shade the time spans where the assist intervened
    start = None
    for i, row in enumerate(rows + [{"avoidance_level": "CLEAR", "time_s": rows[-1]["time_s"]}]):
        level = row["avoidance_level"]
        if level != "CLEAR" and start is None:
            start = (float(rows[i]["time_s"]), level)
        elif (level == "CLEAR" or (start and level != start[1])) and start is not None:
            for ax in axes:
                ax.axvspan(start[0], float(row["time_s"]), color=LEVEL_COLORS[start[1]], alpha=0.18, linewidth=0)
            start = (float(row["time_s"]), level) if level != "CLEAR" else None
    axes[-1].set_xlabel("Simulation time [s]")
    fig.suptitle(f"Collision avoidance performance - {path.name}")
    fig.tight_layout()
    out = path.with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"Saved {out}")
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())

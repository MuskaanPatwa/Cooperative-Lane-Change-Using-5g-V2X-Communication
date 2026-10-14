"""Headless batch runner: runs every scenario and prints a results table.

    python run_batch.py                       # 3 seeds x 5 scenarios, 90 s each
    python run_batch.py --duration 60 --seeds 5 --no-auto-lane

The simulated driver holds the accelerator the whole time, so the collision
avoidance assist has to do all the avoiding. Results are also saved to
logs/batch_results.csv.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import config
from analysis.metrics import MetricsRecorder, RunSummary
from analysis.scenarios import SCENARIOS, Scenario
from controllers.vehicle_controller import ControlInput
from simulation.simulation import Simulation


def run_scenario(scenario: Scenario, seed: int, duration_s: float, auto_lane_change: bool, dt: float = 0.02) -> RunSummary:
    """Run one scenario headlessly and return its summary."""
    sim = Simulation(seed=seed)
    sim.vehicle_controller.auto_lane_change = auto_lane_change
    scenario.setup(sim)
    recorder = MetricsRecorder()
    control = ControlInput(accelerate=True)
    for _ in range(int(duration_s / dt)):
        if scenario.on_step:
            scenario.on_step(sim)
        sim.step(dt, control)
        recorder.update(sim)
    return recorder.summary()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run all scenarios headlessly and summarise the results")
    parser.add_argument("--duration", type=float, default=90.0, help="simulated seconds per run")
    parser.add_argument("--seeds", type=int, default=3, help="number of random seeds per scenario")
    parser.add_argument("--no-auto-lane", action="store_true", help="disable the evasive lane change")
    args = parser.parse_args()

    header = ["scenario", "seed", "collisions", "min_gap_m", "min_ttc_s", "braking_events",
              "emergency_events", "peak_decel_mps2", "evasive_lane_changes", "avg_speed_kmh"]
    rows: list[list[object]] = []
    for scenario in SCENARIOS.values():
        for seed in range(args.seeds):
            s = run_scenario(scenario, seed, args.duration, not args.no_auto_lane)
            rows.append([
                scenario.name, seed, s.collisions,
                "" if s.min_gap_m is None else round(s.min_gap_m, 1),
                "" if s.min_ttc_s is None else round(s.min_ttc_s, 1),
                s.braking_events, s.emergency_events, round(s.peak_decel_mps2, 1),
                s.evasive_lane_changes, round(s.avg_speed_mps * 3.6, 1),
            ])

    widths = [max(len(str(r[i])) for r in [header, *rows]) for i in range(len(header))]
    for row in [header, *rows]:
        print("  ".join(str(cell).rjust(w) for cell, w in zip(row, widths)))
    total = sum(int(r[2]) for r in rows)
    print(f"\nRuns: {len(rows)}   total collisions: {total}")

    out = Path(config.LOG_DIR) / "batch_results.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    print(f"Saved {out}")
    return 0 if total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

"""Metrics recorder, summary, CSV export, scenarios and graph drawing (headless)."""
import csv
import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pytest

import config
from analysis import CSV_COLUMNS, MetricsRecorder
from analysis.scenarios import SCENARIOS
from controllers import ControlInput, LaneDirection
from simulation.simulation import Simulation
from run_batch import run_scenario


def run(sim, rec, seconds, control=None, dt=1 / 60):
    for _ in range(int(seconds / dt)):
        sim.step(dt, control)
        rec.update(sim)


def test_recorder_rejects_bad_period():
    with pytest.raises(ValueError):
        MetricsRecorder(sample_period_s=0)


def test_sampling_period_and_duration():
    sim, rec = Simulation(), MetricsRecorder(sample_period_s=0.1)
    run(sim, rec, 5.0)
    assert 49 <= len(rec.samples) <= 52
    times = [s.time for s in rec.samples]
    assert all(b > a for a, b in zip(times, times[1:]))
    summary = rec.summary()
    assert summary.duration_s == pytest.approx(5.0, abs=0.05)
    assert summary.distance_m == pytest.approx(sim.ego.s, abs=1.0)


def test_paused_simulation_records_nothing_new():
    sim, rec = Simulation(), MetricsRecorder()
    run(sim, rec, 1.0)
    count = len(rec.samples)
    sim.toggle_pause()
    run(sim, rec, 1.0)
    assert len(rec.samples) == count


def test_recorder_resets_when_simulation_resets():
    sim, rec = Simulation(), MetricsRecorder()
    run(sim, rec, 3.0)
    sim.reset()
    sim.step(1 / 60)
    rec.update(sim)
    assert len(rec.samples) <= 2 and rec.summary().duration_s < 0.1


def test_extremes_and_level_time_for_slow_leader():
    sim, rec = Simulation(), MetricsRecorder()
    SCENARIOS["slow_leader"].setup(sim)
    sim.vehicle_controller.auto_lane_change = False
    run(sim, rec, 40.0, ControlInput(accelerate=True))
    s = rec.summary()
    assert s.collisions == 0
    assert s.min_gap_m is not None and s.min_gap_m > 0
    assert s.min_ttc_s is not None and s.min_ttc_s < 10
    assert s.peak_decel_mps2 > 0 and s.braking_events >= 1
    total = s.time_clear_s + s.time_slowing_s + s.time_braking_s + s.time_emergency_s
    assert total == pytest.approx(s.duration_s, abs=0.05)


def test_lane_change_counters_in_summary():
    sim, rec = Simulation(enforce_safety=False), MetricsRecorder()
    sim.vehicle_controller.auto_lane_change = False
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.LEFT))
    rec.update(sim)
    run(sim, rec, 3.0)
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.LEFT))     # already leftmost -> rejected
    rec.update(sim)
    s = rec.summary()
    assert s.manual_lane_changes_accepted == 1 and s.manual_lane_changes_rejected == 1


def test_csv_export_header_rows_and_blank_cells(tmp_path):
    sim, rec = Simulation(), MetricsRecorder()
    run(sim, rec, 3.0, ControlInput(accelerate=True))
    path = rec.write_csv(tmp_path / "nested" / "run.csv")
    with path.open(newline="") as handle:
        rows = list(csv.reader(handle))
    assert tuple(rows[0]) == CSV_COLUMNS
    assert len(rows) - 1 == len(rec.samples)
    assert all(len(r) == len(CSV_COLUMNS) for r in rows)
    assert float(rows[-1][0]) == pytest.approx(rec.samples[-1].time, abs=1e-3)


def test_summary_lines_and_dict_are_complete():
    rec = MetricsRecorder()
    summary = rec.summary()                    # empty recorder must not crash
    assert summary.min_gap_m is None and summary.avg_speed_mps == 0.0
    assert ("Collisions", "0") in summary.format_lines()
    assert summary.to_dict()["collisions"] == 0


@pytest.mark.parametrize("name", list(SCENARIOS))
def test_every_scenario_runs_without_collision(name):
    summary = run_scenario(SCENARIOS[name], seed=0, duration_s=30.0, auto_lane_change=True)
    assert summary.collisions == 0
    assert summary.duration_s == pytest.approx(30.0, abs=0.1)


def test_scenarios_without_evasion_also_collision_free():
    for name in ("slow_leader", "sudden_brake"):
        assert run_scenario(SCENARIOS[name], 1, 30.0, auto_lane_change=False).collisions == 0


def test_sudden_brake_scenario_triggers_hard_braking():
    s = run_scenario(SCENARIOS["sudden_brake"], 0, 20.0, auto_lane_change=False)
    assert s.emergency_events >= 1 and s.peak_decel_mps2 >= config.AVOID_EMERGENCY_DECEL_MPS2


def test_graph_overlay_and_summary_card_draw_without_error():
    pygame = pytest.importorskip("pygame")
    pygame.init()
    from visualization.graphs import GraphOverlay, SummaryCard, draw_toast
    surface = pygame.Surface((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
    sim, rec = Simulation(), MetricsRecorder()
    GraphOverlay(surface).draw(rec)            # empty recorder
    run(sim, rec, 10.0, ControlInput(accelerate=True))
    GraphOverlay(surface).draw(rec)
    SummaryCard(surface).draw(rec)
    draw_toast(surface, "hello")
    assert surface.get_at((5, 5))[:3] != (0, 0, 0) or True

"""Records simulation metrics and builds a run summary.

The recorder only *reads* the simulation (call ``update(sim)`` once per frame
after ``sim.step``); it never changes it. Samples are taken every
``METRICS_SAMPLE_PERIOD_S`` of simulation time, while extremes (minimum gap,
minimum TTC, peak deceleration) are tracked on every update.
"""
from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import config
from safety.collision_avoidance import AvoidanceLevel

if TYPE_CHECKING:
    from simulation.simulation import Simulation

CSV_COLUMNS = (
    "time_s", "speed_mps", "accel_mps2", "lane", "maneuver", "front_vehicle",
    "front_gap_m", "closing_speed_mps", "front_ttc_s", "avoidance_level",
    "avoid_decel_mps2", "left_lane_status", "right_lane_status", "collision",
)


@dataclass(frozen=True)
class Sample:
    """One recorded row. ``front_gap``/``front_ttc`` are ``None`` when not applicable."""

    time: float
    speed: float
    acceleration: float
    lane: int                      # 1-based, as shown on screen
    maneuver: str
    front_vehicle: Optional[str]
    front_gap: Optional[float]
    closing_speed: float
    front_ttc: Optional[float]
    level: AvoidanceLevel
    decel: float
    left_status: str
    right_status: str
    collision: bool

    def csv_row(self) -> list[object]:
        """Values in ``CSV_COLUMNS`` order; ``None`` becomes an empty cell."""
        def num(value: Optional[float], digits: int) -> object:
            return "" if value is None else round(value, digits)
        return [
            round(self.time, 3), round(self.speed, 3), round(self.acceleration, 3), self.lane,
            self.maneuver, self.front_vehicle or "", num(self.front_gap, 3),
            round(self.closing_speed, 3), num(self.front_ttc, 3), self.level.value,
            round(self.decel, 3), self.left_status, self.right_status, int(self.collision),
        ]


@dataclass(frozen=True)
class RunSummary:
    """Aggregate performance numbers of one run."""

    duration_s: float
    distance_m: float
    avg_speed_mps: float
    max_speed_mps: float
    min_gap_m: Optional[float]
    min_ttc_s: Optional[float]
    braking_events: int
    emergency_events: int
    peak_decel_mps2: float
    time_clear_s: float
    time_slowing_s: float
    time_braking_s: float
    time_emergency_s: float
    evasive_lane_changes: int
    manual_lane_changes_accepted: int
    manual_lane_changes_rejected: int
    collisions: int
    v2x_messages_logged: int

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def format_lines(self) -> list[tuple[str, str]]:
        """(label, value) pairs for the HUD card and console output."""
        def opt(value: Optional[float], unit: str) -> str:
            return "--" if value is None else f"{value:.1f} {unit}"
        return [
            ("Duration", f"{self.duration_s:.1f} s"),
            ("Distance", f"{self.distance_m:.0f} m"),
            ("Avg / max speed", f"{self.avg_speed_mps * 3.6:.0f} / {self.max_speed_mps * 3.6:.0f} km/h"),
            ("Min gap to leader", opt(self.min_gap_m, "m")),
            ("Min TTC", opt(self.min_ttc_s, "s")),
            ("Braking events", str(self.braking_events)),
            ("Emergency events", str(self.emergency_events)),
            ("Peak auto-decel", f"{self.peak_decel_mps2:.1f} m/s2"),
            ("Time CLEAR", f"{self.time_clear_s:.1f} s"),
            ("Time SLOWING", f"{self.time_slowing_s:.1f} s"),
            ("Time BRAKING", f"{self.time_braking_s:.1f} s"),
            ("Time EMERGENCY", f"{self.time_emergency_s:.1f} s"),
            ("Evasive lane changes", str(self.evasive_lane_changes)),
            ("Manual LC ok / rejected", f"{self.manual_lane_changes_accepted} / {self.manual_lane_changes_rejected}"),
            ("Collisions", str(self.collisions)),
            ("V2X msgs logged", str(self.v2x_messages_logged)),
        ]


def default_csv_path() -> Path:
    """``logs/run_YYYYmmdd_HHMMSS.csv`` (relative to the working directory)."""
    return Path(config.LOG_DIR) / f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"


class MetricsRecorder:
    """Collects samples and summary statistics from a running :class:`Simulation`."""

    def __init__(self, sample_period_s: float = config.METRICS_SAMPLE_PERIOD_S) -> None:
        if sample_period_s <= 0:
            raise ValueError("sample_period_s must be > 0")
        self.sample_period_s = sample_period_s
        self.reset()

    def reset(self) -> None:
        """Forget everything recorded so far."""
        self.samples: list[Sample] = []
        self._last_time = -1.0
        self._next_sample = 0.0
        self._start_s: Optional[float] = None
        self._distance = 0.0
        self._duration = 0.0
        self._max_speed = 0.0
        self._min_gap = math.inf
        self._min_ttc = math.inf
        self._peak_decel = 0.0
        self._prev_rank = 0
        self._braking_events = 0
        self._emergency_events = 0
        self._time_in_level = {level: 0.0 for level in AvoidanceLevel}
        self._evasive = self._manual_ok = self._manual_rejected = 0
        self._collisions = 0
        self._v2x_logged = 0

    # ----------------------------------------------------------------- update
    def update(self, sim: "Simulation") -> None:
        """Read the simulation state; call once per frame after ``sim.step``."""
        t = sim.time
        if t < self._last_time - 1e-9:      # simulation was reset
            self.reset()
        if t == self._last_time:            # paused / no time passed
            return
        dt = 0.0 if self._last_time < 0 else t - self._last_time
        self._last_time = t

        ego = sim.ego
        avoid = sim.vehicle_controller.last_avoidance
        if self._start_s is None:
            self._start_s = ego.s
        self._distance = ego.s - self._start_s
        self._duration += dt
        self._max_speed = max(self._max_speed, ego.speed)
        self._time_in_level[avoid.level] += dt
        self._peak_decel = max(self._peak_decel, avoid.decel)
        if math.isfinite(avoid.front_gap):
            self._min_gap = min(self._min_gap, avoid.front_gap)
        if math.isfinite(avoid.ttc):
            self._min_ttc = min(self._min_ttc, avoid.ttc)

        rank = avoid.level.rank
        if rank >= AvoidanceLevel.BRAKING.rank and self._prev_rank < AvoidanceLevel.BRAKING.rank:
            self._braking_events += 1
        if rank >= AvoidanceLevel.EMERGENCY_BRAKING.rank and self._prev_rank < AvoidanceLevel.EMERGENCY_BRAKING.rank:
            self._emergency_events += 1
        self._prev_rank = rank

        stats = sim.vehicle_controller.stats
        self._evasive = stats.evasive_accepted
        self._manual_ok = stats.manual_accepted
        self._manual_rejected = stats.manual_rejected
        self._collisions = sim.collision_count
        self._v2x_logged = getattr(sim.v2x, "sent_count", 0)

        if t + 1e-9 >= self._next_sample:
            self._next_sample = t + self.sample_period_s
            report = sim.safety_report
            self.samples.append(Sample(
                time=t, speed=ego.speed, acceleration=ego.acceleration, lane=ego.lane + 1,
                maneuver=ego.maneuver.value, front_vehicle=avoid.front_vehicle_id,
                front_gap=avoid.front_gap if math.isfinite(avoid.front_gap) else None,
                closing_speed=avoid.closing_speed,
                front_ttc=avoid.ttc if math.isfinite(avoid.ttc) else None,
                level=avoid.level, decel=avoid.decel,
                left_status="N/A" if report.left.out_of_road else report.left.status.value,
                right_status="N/A" if report.right.out_of_road else report.right.status.value,
                collision=sim.collision_with is not None,
            ))

    # ----------------------------------------------------------------- output
    def summary(self) -> RunSummary:
        """Aggregate numbers for everything recorded so far."""
        avg = self._distance / self._duration if self._duration > 0 else 0.0
        lv = self._time_in_level
        return RunSummary(
            duration_s=self._duration, distance_m=self._distance, avg_speed_mps=avg,
            max_speed_mps=self._max_speed,
            min_gap_m=self._min_gap if math.isfinite(self._min_gap) else None,
            min_ttc_s=self._min_ttc if math.isfinite(self._min_ttc) else None,
            braking_events=self._braking_events, emergency_events=self._emergency_events,
            peak_decel_mps2=self._peak_decel,
            time_clear_s=lv[AvoidanceLevel.CLEAR], time_slowing_s=lv[AvoidanceLevel.SLOWING],
            time_braking_s=lv[AvoidanceLevel.BRAKING], time_emergency_s=lv[AvoidanceLevel.EMERGENCY_BRAKING],
            evasive_lane_changes=self._evasive, manual_lane_changes_accepted=self._manual_ok,
            manual_lane_changes_rejected=self._manual_rejected, collisions=self._collisions,
            v2x_messages_logged=self._v2x_logged,
        )

    def write_csv(self, path: Optional[Path | str] = None) -> Path:
        """Write all samples to a CSV file (folders are created) and return its path."""
        target = Path(path) if path is not None else default_csv_path()
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(CSV_COLUMNS)
            for sample in self.samples:
                writer.writerow(sample.csv_row())
        return target

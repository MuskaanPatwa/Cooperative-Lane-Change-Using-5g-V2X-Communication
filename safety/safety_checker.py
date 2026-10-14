"""Basic lane-change safety checks (Phase 1).

What is IMPLEMENTED here:
  1. Road-boundary check (does the target lane exist?).
  2. Target-lane occupancy (is a vehicle body already overlapping the slot?).
  3. Bumper-to-bumper distance to the nearest vehicle ahead and behind.
  4. Relative (closing) speed to those two vehicles.
  5. A preliminary constant-speed time-to-collision (TTC) estimate.

What is NOT implemented: trajectory prediction, braking/acceleration models,
reaction times, sensor/communication uncertainty, multi-vehicle interaction, or
any validation against real-world safety standards. The result is a *basic gap
check*, not a collision-prediction model and not a certified safety system.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Optional, Sequence

import config
from simulation.road import Road

if TYPE_CHECKING:  # avoid runtime import cycles with the vehicles package
    from vehicles.vehicle import Vehicle


class SafetyStatus(str, Enum):
    """Outcome of a lane-change safety evaluation."""

    SAFE = "SAFE"
    UNSAFE = "UNSAFE"
    WAITING_FOR_GAP = "WAITING_FOR_GAP"


@dataclass(frozen=True)
class SafetyConfig:
    """Thresholds used by :class:`SafetyChecker`."""

    min_front_gap_m: float = config.SAFETY_MIN_FRONT_GAP_M
    min_rear_gap_m: float = config.SAFETY_MIN_REAR_GAP_M
    min_ttc_s: float = config.SAFETY_MIN_TTC_S


@dataclass(frozen=True)
class LaneGaps:
    """Gaps between a subject vehicle and its neighbours in one lane.

    Gaps are bumper-to-bumper distances in metres (``math.inf`` = no vehicle).
    Closing speeds are positive when the gap is shrinking.
    TTC values are in seconds (``math.inf`` = not closing).
    """

    lane: int
    front_vehicle_id: Optional[str] = None
    front_gap: float = math.inf
    front_closing_speed: float = 0.0
    front_ttc: float = math.inf
    rear_vehicle_id: Optional[str] = None
    rear_gap: float = math.inf
    rear_closing_speed: float = 0.0
    rear_ttc: float = math.inf
    overlapping_vehicle_id: Optional[str] = None


@dataclass(frozen=True)
class SafetyResult:
    """Result of evaluating a lane change into ``target_lane``."""

    status: SafetyStatus
    target_lane: int
    gaps: Optional[LaneGaps]
    reasons: tuple[str, ...] = ()
    out_of_road: bool = False

    def summary(self) -> str:
        """Short human-readable description."""
        if self.out_of_road:
            return "N/A (road edge)"
        return self.status.value


@dataclass(frozen=True)
class LaneSafetyReport:
    """Everything the HUD shows: own-lane gaps plus left/right lane checks."""

    own_lane: LaneGaps
    left: SafetyResult
    right: SafetyResult


# --------------------------------------------------------------------------- math
def bumper_gap(rear: "Vehicle", front: "Vehicle") -> float:
    """Bumper-to-bumper gap (m) between a rear and a front vehicle.

    Negative values mean the bodies overlap longitudinally.
    """
    return (front.s - rear.s) - (front.length + rear.length) / 2.0


def time_to_collision(gap_m: float, closing_speed_mps: float) -> float:
    """Constant-speed TTC estimate in seconds.

    Returns ``math.inf`` when the vehicles are not closing, and ``0.0`` when
    they already overlap (``gap_m <= 0``) while closing or touching.
    """
    if gap_m <= 0.0:
        return 0.0
    if closing_speed_mps <= config.SAFETY_CLOSING_SPEED_EPS_MPS:
        return math.inf
    return gap_m / closing_speed_mps


def compute_lane_gaps(subject: "Vehicle", vehicles: Sequence["Vehicle"], lane: int) -> LaneGaps:
    """Find the nearest vehicle ahead/behind ``subject`` among those occupying ``lane``."""
    front: Optional["Vehicle"] = None
    rear: Optional["Vehicle"] = None
    front_gap = rear_gap = math.inf
    overlapping: Optional[str] = None

    for other in vehicles:
        if other is subject or other.vehicle_id == subject.vehicle_id:
            continue
        if lane not in other.occupied_lanes():
            continue
        delta = other.s - subject.s
        gap = abs(delta) - (subject.length + other.length) / 2.0
        if gap < 0.0:
            overlapping = overlapping or other.vehicle_id
        elif delta >= 0.0:
            if gap < front_gap:
                front, front_gap = other, gap
        elif gap < rear_gap:
            rear, rear_gap = other, gap

    front_closing = (subject.speed - front.speed) if front else 0.0
    rear_closing = (rear.speed - subject.speed) if rear else 0.0
    return LaneGaps(
        lane=lane,
        front_vehicle_id=front.vehicle_id if front else None,
        front_gap=front_gap,
        front_closing_speed=front_closing,
        front_ttc=time_to_collision(front_gap, front_closing) if front else math.inf,
        rear_vehicle_id=rear.vehicle_id if rear else None,
        rear_gap=rear_gap,
        rear_closing_speed=rear_closing,
        rear_ttc=time_to_collision(rear_gap, rear_closing) if rear else math.inf,
        overlapping_vehicle_id=overlapping,
    )


# ------------------------------------------------------------------------ checker
class SafetyChecker:
    """Evaluates a prospective lane change with the basic checks listed above."""

    def __init__(self, safety_config: SafetyConfig | None = None) -> None:
        self.config = safety_config or SafetyConfig()

    def evaluate_lane_change(
        self,
        subject: "Vehicle",
        target_lane: int,
        vehicles: Sequence["Vehicle"],
        road: Road,
    ) -> SafetyResult:
        """Return SAFE / UNSAFE / WAITING_FOR_GAP for moving ``subject`` to ``target_lane``.

        UNSAFE: lane outside the road, occupied slot, or TTC below the minimum.
        WAITING_FOR_GAP: nobody imminent, but a gap is below its minimum
        (a gap that may open up if the traffic evolves).
        SAFE: none of the above.
        """
        if not road.is_valid_lane(target_lane):
            return SafetyResult(
                SafetyStatus.UNSAFE, target_lane, None,
                (f"lane {target_lane!r} is outside the road boundary",), out_of_road=True,
            )

        gaps = compute_lane_gaps(subject, vehicles, target_lane)
        cfg = self.config
        reasons: list[str] = []
        unsafe = waiting = False

        if gaps.overlapping_vehicle_id:
            unsafe = True
            reasons.append(f"target slot occupied by {gaps.overlapping_vehicle_id}")
        if gaps.front_ttc < cfg.min_ttc_s:
            unsafe = True
            reasons.append(f"front TTC {gaps.front_ttc:.1f}s < {cfg.min_ttc_s:.1f}s")
        if gaps.rear_ttc < cfg.min_ttc_s:
            unsafe = True
            reasons.append(f"rear TTC {gaps.rear_ttc:.1f}s < {cfg.min_ttc_s:.1f}s")
        if gaps.front_gap < cfg.min_front_gap_m and not gaps.overlapping_vehicle_id:
            waiting = True
            reasons.append(f"front gap {gaps.front_gap:.1f}m < {cfg.min_front_gap_m:.1f}m")
        if gaps.rear_gap < cfg.min_rear_gap_m and not gaps.overlapping_vehicle_id:
            waiting = True
            reasons.append(f"rear gap {gaps.rear_gap:.1f}m < {cfg.min_rear_gap_m:.1f}m")

        if unsafe:
            status = SafetyStatus.UNSAFE
        elif waiting:
            status = SafetyStatus.WAITING_FOR_GAP
        else:
            status = SafetyStatus.SAFE
        return SafetyResult(status, target_lane, gaps, tuple(reasons))

    def report(self, subject: "Vehicle", vehicles: Sequence["Vehicle"], road: Road) -> LaneSafetyReport:
        """Own-lane gaps plus the safety result for both neighbouring lanes."""
        return LaneSafetyReport(
            own_lane=compute_lane_gaps(subject, vehicles, subject.lane),
            left=self.evaluate_lane_change(subject, subject.lane - 1, vehicles, road),
            right=self.evaluate_lane_change(subject, subject.lane + 1, vehicles, road),
        )

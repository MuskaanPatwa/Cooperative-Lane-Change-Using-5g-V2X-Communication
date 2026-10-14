"""Base vehicle model: kinematics, lane state and smooth lane transitions."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import config
from simulation.road import Road


class ManeuverState(str, Enum):
    """Lateral maneuver state of a vehicle."""

    CRUISING = "CRUISING"
    LANE_CHANGE_LEFT = "LANE_CHANGE_LEFT"
    LANE_CHANGE_RIGHT = "LANE_CHANGE_RIGHT"


class LaneChangeError(ValueError):
    """Raised when a lane change cannot be started on a vehicle."""


def _smoothstep(t: float) -> float:
    """Ease-in/ease-out curve: 0 -> 1 with zero slope at both ends."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


@dataclass(eq=False)
class Vehicle:
    """A vehicle on the road.

    ``s`` is the longitudinal position of the vehicle centre (m) and
    ``lateral`` the lateral position of its centre (m from the left road edge).
    ``lane`` is the lane the vehicle is leaving/currently assigned to; while a
    lane change is in progress ``target_lane`` holds the destination and the
    vehicle occupies both lanes (see :meth:`occupied_lanes`).
    Identity comparison is used (``eq=False``) so vehicles are hashable.
    """

    vehicle_id: str
    lane: int
    s: float = 0.0
    speed: float = 20.0
    acceleration: float = 0.0
    length: float = config.VEHICLE_LENGTH_M
    width: float = config.VEHICLE_WIDTH_M
    color: config.Color = (200, 200, 200)
    min_speed: float = 0.0
    max_speed: float = 60.0
    road: Road = field(default_factory=Road)
    lane_change_duration: float = config.LANE_CHANGE_DURATION_S
    target_lane: Optional[int] = None
    maneuver: ManeuverState = ManeuverState.CRUISING
    lateral: float = field(init=False, default=0.0)
    lateral_speed: float = field(init=False, default=0.0)
    _lc_elapsed: float = field(init=False, default=0.0, repr=False)
    _lc_start_lateral: float = field(init=False, default=0.0, repr=False)

    def __post_init__(self) -> None:
        if not self.vehicle_id:
            raise ValueError("vehicle_id must be a non-empty string")
        if not self.road.is_valid_lane(self.lane):
            raise ValueError(
                f"Vehicle {self.vehicle_id}: lane {self.lane!r} is outside the road "
                f"(valid lanes: 0..{self.road.num_lanes - 1})"
            )
        if self.length <= 0 or self.width <= 0:
            raise ValueError(f"Vehicle {self.vehicle_id}: length and width must be positive")
        if self.width > self.road.lane_width:
            raise ValueError(f"Vehicle {self.vehicle_id}: wider than a lane")
        if self.min_speed > self.max_speed:
            raise ValueError(f"Vehicle {self.vehicle_id}: min_speed exceeds max_speed")
        if self.lane_change_duration <= 0:
            raise ValueError(f"Vehicle {self.vehicle_id}: lane_change_duration must be > 0")
        self.speed = self._clamp_speed(self.speed)
        self.lateral = self.road.lane_center(self.lane)
        self.target_lane = None
        self.maneuver = ManeuverState.CRUISING

    # ------------------------------------------------------------------ state
    @property
    def lane_change_in_progress(self) -> bool:
        """True while a lateral transition is running."""
        return self.target_lane is not None

    @property
    def lane_change_progress(self) -> float:
        """Transition progress in [0, 1]; 0 when no lane change is running."""
        if not self.lane_change_in_progress:
            return 0.0
        return min(1.0, self._lc_elapsed / self.lane_change_duration)

    @property
    def front_s(self) -> float:
        """Longitudinal position of the front bumper."""
        return self.s + self.length / 2.0

    @property
    def rear_s(self) -> float:
        """Longitudinal position of the rear bumper."""
        return self.s - self.length / 2.0

    def occupied_lanes(self) -> tuple[int, ...]:
        """Lanes the vehicle occupies; both lanes while changing lane."""
        if self.target_lane is not None:
            return (self.lane, self.target_lane)
        return (self.lane,)

    # ------------------------------------------------------------ longitudinal
    def _clamp_speed(self, speed: float) -> float:
        return max(self.min_speed, min(self.max_speed, speed))

    def set_acceleration(self, acceleration: float) -> None:
        """Set the commanded acceleration, limited to the physical limits."""
        if not math.isfinite(acceleration):
            raise ValueError(f"acceleration must be finite, got {acceleration}")
        self.acceleration = max(-config.MAX_DECEL_MPS2, min(config.MAX_ACCEL_MPS2, acceleration))

    # ----------------------------------------------------------------- lateral
    def start_lane_change(self, target_lane: int) -> None:
        """Begin a smooth transition to an adjacent lane.

        Raises:
            LaneChangeError: if a transition is running, the lane is outside
                the road, or the lane is not directly adjacent.
        """
        if self.lane_change_in_progress:
            raise LaneChangeError(f"{self.vehicle_id}: a lane change is already in progress")
        if not self.road.is_valid_lane(target_lane):
            raise LaneChangeError(
                f"{self.vehicle_id}: target lane {target_lane!r} is outside the road"
            )
        if abs(target_lane - self.lane) != 1:
            raise LaneChangeError(
                f"{self.vehicle_id}: lane {target_lane} is not adjacent to lane {self.lane}"
            )
        self.target_lane = target_lane
        self.maneuver = (
            ManeuverState.LANE_CHANGE_LEFT if target_lane < self.lane
            else ManeuverState.LANE_CHANGE_RIGHT
        )
        self._lc_elapsed = 0.0
        self._lc_start_lateral = self.lateral

    def _advance_lane_change(self, dt: float) -> None:
        if self.target_lane is None:
            self.lateral_speed = 0.0
            return
        self._lc_elapsed += dt
        progress = min(1.0, self._lc_elapsed / self.lane_change_duration)
        end = self.road.lane_center(self.target_lane)
        new_lateral = self._lc_start_lateral + (end - self._lc_start_lateral) * _smoothstep(progress)
        new_lateral = self.road.clamp_lateral(new_lateral, self.width)
        self.lateral_speed = (new_lateral - self.lateral) / dt
        self.lateral = new_lateral
        if progress >= 1.0:
            self.lane = self.target_lane
            self.target_lane = None
            self.maneuver = ManeuverState.CRUISING
            self.lateral = end
            self.lateral_speed = 0.0
            self._lc_elapsed = 0.0

    # ------------------------------------------------------------------ update
    def update(self, dt: float) -> None:
        """Advance the vehicle by ``dt`` seconds (frame-rate independent)."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError(f"dt must be a finite, non-negative number, got {dt}")
        if dt == 0:
            return
        new_speed = self._clamp_speed(self.speed + self.acceleration * dt)
        self.s += 0.5 * (self.speed + new_speed) * dt
        self.speed = new_speed
        self._advance_lane_change(dt)

    # ---------------------------------------------------------------- utilities
    def place(self, lane: int, s: float, speed: float) -> None:
        """Teleport the vehicle (used for traffic recycling); cancels any lane change."""
        if not self.road.is_valid_lane(lane):
            raise ValueError(f"lane {lane!r} is outside the road")
        self.lane = lane
        self.s = s
        self.speed = self._clamp_speed(speed)
        self.acceleration = 0.0
        self.target_lane = None
        self.maneuver = ManeuverState.CRUISING
        self.lateral = self.road.lane_center(lane)
        self.lateral_speed = 0.0
        self._lc_elapsed = 0.0

    def overlaps(self, other: "Vehicle") -> bool:
        """True if the two vehicle bodies (axis-aligned rectangles) intersect."""
        return (
            abs(self.s - other.s) < (self.length + other.length) / 2.0
            and abs(self.lateral - other.lateral) < (self.width + other.width) / 2.0
        )

"""Automatic surrounding traffic.

IMPORTANT: this is a deliberately simple longitudinal model (speed holding plus
a basic gap-based braking rule so cars do not drive through each other). It is
NOT a validated car-following model (e.g. IDM) and NOT an autonomous-driving
stack. Traffic vehicles do not change lanes in Phase 1.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import config
from safety.safety_checker import compute_lane_gaps
from vehicles.vehicle import Vehicle


@dataclass(eq=False)
class TrafficVehicle(Vehicle):
    """Background vehicle that tries to hold ``desired_speed``."""

    min_speed: float = config.TRAFFIC_MIN_SPEED_MPS
    max_speed: float = config.TRAFFIC_MAX_SPEED_MPS
    desired_speed: float = 20.0
    time_headway_s: float = config.TRAFFIC_TIME_HEADWAY_S
    standstill_gap_m: float = config.TRAFFIC_STANDSTILL_GAP_M

    def __post_init__(self) -> None:
        super().__post_init__()
        self.desired_speed = max(self.min_speed, min(self.max_speed, self.desired_speed))

    def compute_acceleration(self, vehicles: Sequence[Vehicle]) -> float:
        """Acceleration from a basic speed-holding + gap-keeping rule."""
        acceleration = config.TRAFFIC_SPEED_GAIN * (self.desired_speed - self.speed)

        nearest_gap = math.inf
        closing_speed = 0.0
        for lane in self.occupied_lanes():
            gaps = compute_lane_gaps(self, vehicles, lane)
            if gaps.front_gap < nearest_gap:
                nearest_gap = gaps.front_gap
                closing_speed = gaps.front_closing_speed

        if math.isfinite(nearest_gap):
            leader_speed = self.speed - closing_speed
            desired_gap = self.standstill_gap_m + self.time_headway_s * self.speed
            if nearest_gap < 2.0 * desired_gap:
                follow = (
                    config.TRAFFIC_GAP_GAIN * (nearest_gap - desired_gap)
                    + config.TRAFFIC_REL_SPEED_GAIN * (leader_speed - self.speed)
                )
                acceleration = min(acceleration, follow)
            if nearest_gap < 0.5 * self.standstill_gap_m:
                acceleration = -config.MAX_DECEL_MPS2

        return max(-config.MAX_DECEL_MPS2, min(config.MAX_ACCEL_MPS2, acceleration))

    def update_acceleration(self, vehicles: Sequence[Vehicle]) -> None:
        """Recompute and store the commanded acceleration."""
        self.set_acceleration(self.compute_acceleration(vehicles))

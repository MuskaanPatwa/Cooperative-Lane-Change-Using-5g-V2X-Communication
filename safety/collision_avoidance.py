"""Collision-avoidance assist for the EGO vehicle (automatic slowing / braking).

How it works (constant-speed-leader model):
  * Take the nearest vehicle ahead in any lane the EGO occupies.
  * If the EGO is closing on it, the deceleration needed to stop closing before
    ``AVOID_STOP_MARGIN_M`` is  a = v_closing^2 / (2 * (gap - margin)).
    The assist brakes with a * AVOID_BRAKE_FACTOR (capped at the physical limit).
  * Independently, if the gap is below a desired time gap it brakes gently so
    the gap reopens (simple gap keeping).
  * If the leader itself is braking (its acceleration is read like a radar's
    relative-acceleration estimate) and is within ~2 desired gaps, the EGO brakes
    at least as hard immediately.

Limits: it only looks forward, assumes the leader keeps its speed, and cannot
prevent a vehicle from hitting the EGO from behind or avoid a leader that
brakes harder than the EGO can. It is a simulation aid, NOT a validated
automotive AEB/ACC system.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Optional, Sequence

import config
from safety.safety_checker import compute_lane_gaps, time_to_collision

if TYPE_CHECKING:
    from vehicles.vehicle import Vehicle


class AvoidanceLevel(str, Enum):
    """Intervention level, from none to hardest braking."""

    CLEAR = "CLEAR"
    SLOWING = "SLOWING"
    BRAKING = "BRAKING"
    EMERGENCY_BRAKING = "EMERGENCY_BRAKING"

    @property
    def rank(self) -> int:
        return list(AvoidanceLevel).index(self)


@dataclass(frozen=True)
class AvoidanceCommand:
    """Result of one assessment."""

    level: AvoidanceLevel = AvoidanceLevel.CLEAR
    decel: float = 0.0                       # m/s^2, positive = braking
    required_decel: float = 0.0              # raw physics requirement, no safety factor
    front_vehicle_id: Optional[str] = None
    front_gap: float = math.inf
    closing_speed: float = 0.0
    ttc: float = math.inf

    @property
    def wants_evasion(self) -> bool:
        """True when braking is active because the EGO is closing on a slower vehicle."""
        return self.decel > 0.0 and self.closing_speed > config.AVOID_CLOSING_EPS_MPS


class CollisionAvoidance:
    """Computes the automatic braking needed to avoid the vehicle ahead."""

    def assess(self, ego: "Vehicle", vehicles: Sequence["Vehicle"]) -> AvoidanceCommand:
        gap, closing, front_id = math.inf, 0.0, None
        for lane in ego.occupied_lanes():
            gaps = compute_lane_gaps(ego, vehicles, lane)
            if gaps.front_gap < gap:
                gap, closing, front_id = gaps.front_gap, gaps.front_closing_speed, gaps.front_vehicle_id
        if front_id is None:
            return AvoidanceCommand()

        max_decel = config.MAX_DECEL_MPS2
        required = 0.0
        if closing > config.AVOID_CLOSING_EPS_MPS:
            usable = gap - config.AVOID_STOP_MARGIN_M
            required = max_decel if usable <= 0.1 else closing ** 2 / (2.0 * usable)

        follow = 0.0
        desired_gap = max(config.AVOID_FOLLOW_MIN_GAP_M, ego.speed * config.AVOID_FOLLOW_TIME_S)
        if gap < desired_gap and closing > -config.AVOID_CLOSING_EPS_MPS:
            follow = 0.5 + 2.0 * (desired_gap - gap) / desired_gap

        # Feed-forward: a leader that is braking hard must be matched at once, before the gap
        # has visibly started to close (like the relative-acceleration term in a real ACC).
        feed = 0.0
        leader = next((v for v in vehicles if v.vehicle_id == front_id), None)
        lead_brake = max(0.0, -leader.acceleration) if leader is not None else 0.0
        if lead_brake > config.AVOID_MIN_ACTIVE_DECEL_MPS2 and gap < config.AVOID_LEAD_BRAKE_RANGE_FACTOR * desired_gap:
            feed = lead_brake * config.AVOID_LEAD_BRAKE_FACTOR

        decel = min(max_decel, max(required * config.AVOID_BRAKE_FACTOR, follow, feed))
        if decel < config.AVOID_MIN_ACTIVE_DECEL_MPS2:
            decel = 0.0

        if decel >= config.AVOID_EMERGENCY_DECEL_MPS2:
            level = AvoidanceLevel.EMERGENCY_BRAKING
        elif decel >= config.AVOID_BRAKING_DECEL_MPS2:
            level = AvoidanceLevel.BRAKING
        elif decel > 0.0:
            level = AvoidanceLevel.SLOWING
        else:
            level = AvoidanceLevel.CLEAR
        return AvoidanceCommand(level, decel, required, front_id, gap, closing, time_to_collision(gap, closing))

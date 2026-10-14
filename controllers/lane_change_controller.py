"""Lane-change request handling.

A request is validated against the road boundaries and the "no transition
already running" rule. The basic safety check is always evaluated; when
``enforce_safety`` is True (the default) any result other than SAFE rejects
the request, otherwise it is only reported. Cooperative
negotiation over V2X is NOT implemented yet.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

from safety.safety_checker import SafetyChecker, SafetyResult, SafetyStatus
from simulation.road import Road
from vehicles.vehicle import LaneChangeError, Vehicle


class LaneDirection(Enum):
    """Direction of a lane-change request (lane indices grow to the right)."""

    LEFT = -1
    RIGHT = 1


@dataclass(frozen=True)
class LaneChangeDecision:
    """Outcome of a lane-change request."""

    accepted: bool
    reason: str
    target_lane: Optional[int] = None
    safety: Optional[SafetyResult] = None


class LaneChangeController:
    """Validates and starts lane changes."""

    def __init__(self, road: Road, safety_checker: SafetyChecker, enforce_safety: bool = False) -> None:
        self.road = road
        self.safety_checker = safety_checker
        self.enforce_safety = enforce_safety

    def request_lane_change(
        self, vehicle: Vehicle, direction: object, vehicles: Sequence[Vehicle]
    ) -> LaneChangeDecision:
        """Handle a request; never raises for invalid requests, returns a rejection."""
        if not isinstance(direction, LaneDirection):
            return LaneChangeDecision(False, f"invalid direction {direction!r}")
        name = direction.name.lower()

        if vehicle.lane_change_in_progress:
            return LaneChangeDecision(False, "a lane change is already in progress")

        target = vehicle.lane + direction.value
        if not self.road.is_valid_lane(target):
            return LaneChangeDecision(False, f"no lane to the {name} (road boundary)", target)

        safety = self.safety_checker.evaluate_lane_change(vehicle, target, vehicles, self.road)
        if self.enforce_safety and safety.status is not SafetyStatus.SAFE:
            detail = "; ".join(safety.reasons) or safety.status.value
            return LaneChangeDecision(False, f"{safety.status.value}: {detail}", target, safety)

        try:
            vehicle.start_lane_change(target)
        except LaneChangeError as error:  # defensive: should be unreachable after checks
            return LaneChangeDecision(False, str(error), target, safety)

        note = "" if safety.status is SafetyStatus.SAFE else f" (basic check says {safety.status.value}; informative only)"
        return LaneChangeDecision(True, f"moving {name}{note}", target, safety)

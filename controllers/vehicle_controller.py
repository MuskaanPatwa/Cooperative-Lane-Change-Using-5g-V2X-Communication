"""Translates driver inputs into EGO commands and applies collision avoidance."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import config
from controllers.lane_change_controller import LaneChangeController, LaneChangeDecision, LaneDirection
from safety.collision_avoidance import AvoidanceCommand, CollisionAvoidance
from safety.safety_checker import SafetyStatus
from vehicles.ego_vehicle import EgoVehicle
from vehicles.vehicle import Vehicle


@dataclass(frozen=True)
class ControlInput:
    """Driver input for one frame: held pedals plus an optional one-shot lane request."""

    accelerate: bool = False
    brake: bool = False
    lane_request: Optional[LaneDirection] = None


@dataclass
class ControllerStats:
    """Counters used by the performance analysis."""

    manual_accepted: int = 0
    manual_rejected: int = 0
    evasive_accepted: int = 0


class VehicleController:
    """Applies :class:`ControlInput` to the EGO vehicle, with collision avoidance on top.

    Automatic braking overrides the driver's throttle (the strongest braking
    wins). If the EGO is closing on a slower vehicle, an evasive lane change is
    attempted when ``auto_lane_change`` is on and a neighbouring lane is SAFE.
    """

    def __init__(
        self,
        lane_change_controller: LaneChangeController,
        avoidance: Optional[CollisionAvoidance] = None,
        auto_lane_change: bool = config.AVOID_AUTO_LANE_CHANGE,
    ) -> None:
        self.lane_change_controller = lane_change_controller
        self.avoidance = avoidance
        self.auto_lane_change = auto_lane_change
        self.last_action = "None"
        self.last_decision: Optional[LaneChangeDecision] = None
        self.last_avoidance = AvoidanceCommand()
        self._evasion_cooldown = 0.0
        self.stats = ControllerStats()

    def reset(self) -> None:
        """Clear remembered state (keeps the auto-lane-change setting)."""
        self.last_action = "None"
        self.last_decision = None
        self.last_avoidance = AvoidanceCommand()
        self._evasion_cooldown = 0.0
        self.stats = ControllerStats()

    def apply(
        self, ego: EgoVehicle, control: ControlInput, vehicles: Sequence[Vehicle], dt: float = 0.0
    ) -> Optional[LaneChangeDecision]:
        """Apply pedals, avoidance and lane requests; return the lane decision if one was made."""
        ego.command(control.accelerate, control.brake)
        if control.brake:
            self.last_action = "Brake"
        elif control.accelerate:
            self.last_action = "Accelerate"

        avoid = AvoidanceCommand()
        if self.avoidance is not None:
            avoid = self.avoidance.assess(ego, vehicles)
            if avoid.decel > 0.0:
                ego.set_acceleration(min(ego.acceleration, -avoid.decel))
                self.last_action = f"AUTO {avoid.level.value} ({avoid.decel:.1f} m/s2)"
        self.last_avoidance = avoid
        self._evasion_cooldown = max(0.0, self._evasion_cooldown - dt)

        if control.lane_request is not None:
            decision = self.lane_change_controller.request_lane_change(ego, control.lane_request, vehicles)
            self.last_decision = decision
            if decision.accepted:
                self.stats.manual_accepted += 1
            else:
                self.stats.manual_rejected += 1
            verb = "accepted" if decision.accepted else "rejected"
            self.last_action = f"Lane {control.lane_request.name} {verb}: {decision.reason}"
            return decision

        if self.auto_lane_change and avoid.wants_evasion and not ego.lane_change_in_progress and self._evasion_cooldown <= 0.0:
            return self._try_evasive_lane_change(ego, vehicles)
        return None

    def _try_evasive_lane_change(self, ego: EgoVehicle, vehicles: Sequence[Vehicle]) -> Optional[LaneChangeDecision]:
        """Try the neighbouring lanes (largest front gap first); silent if none is SAFE."""
        checker = self.lane_change_controller.safety_checker
        candidates = []
        for direction in (LaneDirection.LEFT, LaneDirection.RIGHT):
            result = checker.evaluate_lane_change(ego, ego.lane + direction.value, vehicles, self.lane_change_controller.road)
            if result.status is SafetyStatus.SAFE and result.gaps is not None:
                candidates.append((-result.gaps.front_gap, direction.value, direction))
        for _, _, direction in sorted(candidates, key=lambda c: (c[0], c[1])):
            decision = self.lane_change_controller.request_lane_change(ego, direction, vehicles)
            if decision.accepted:
                self._evasion_cooldown = config.AVOID_EVASION_COOLDOWN_S
                self.stats.evasive_accepted += 1
                self.last_decision = decision
                self.last_action = f"AUTO evasive lane change {direction.name}"
                return decision
        self._evasion_cooldown = config.AVOID_EVASION_RETRY_S
        return None

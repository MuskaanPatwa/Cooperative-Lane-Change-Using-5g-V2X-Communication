"""Top-level simulation: owns vehicles, advances time, links controllers/safety/V2X.

No pygame code lives here, so the whole simulation can be unit-tested headlessly.
"""
from __future__ import annotations

import random
from typing import Optional

import config
from communication.messages import (
    V2XMessage,
    make_emergency_brake_warning,
    make_lane_change_complete,
    make_lane_change_request,
    make_status_broadcast,
)
from communication.v2x_interface import NullV2XChannel, V2XInterface
from controllers.lane_change_controller import LaneChangeController
from controllers.vehicle_controller import ControlInput, VehicleController
from safety.collision_avoidance import AvoidanceLevel, CollisionAvoidance
from safety.safety_checker import LaneSafetyReport, SafetyChecker
from simulation.road import Road
from vehicles.ego_vehicle import EgoVehicle
from vehicles.traffic_vehicle import TrafficVehicle
from vehicles.vehicle import Vehicle


class Simulation:
    """Highway simulation with one EGO vehicle and automatic surrounding traffic."""

    def __init__(
        self,
        seed: int = config.RANDOM_SEED,
        road: Optional[Road] = None,
        v2x: Optional[V2XInterface] = None,
        enforce_safety: bool = config.ENFORCE_SAFETY_ON_LANE_CHANGE,
    ) -> None:
        self.road = road or Road()
        self.seed = seed
        self.v2x: V2XInterface = v2x or NullV2XChannel()
        self.safety_checker = SafetyChecker()
        self.lane_change_controller = LaneChangeController(self.road, self.safety_checker, enforce_safety)
        self.vehicle_controller = VehicleController(
            self.lane_change_controller,
            CollisionAvoidance() if config.AVOID_ENABLED else None,
        )
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        """Restore the initial scenario (same seed -> same scenario)."""
        self._rng = random.Random(self.seed)
        self.time = 0.0
        self.paused = False
        self.ego = EgoVehicle(
            vehicle_id=config.EGO_ID,
            lane=config.EGO_INITIAL_LANE,
            s=config.EGO_INITIAL_S,
            speed=config.EGO_INITIAL_SPEED_MPS,
            road=self.road,
        )
        self.traffic: list[TrafficVehicle] = [
            TrafficVehicle(
                vehicle_id=vid, lane=lane, s=config.EGO_INITIAL_S + offset,
                speed=speed, desired_speed=speed, color=color, road=self.road,
            )
            for vid, lane, offset, speed, color in config.INITIAL_TRAFFIC
        ]
        self.collision_with: Optional[str] = None
        self.collision_count = 0
        self._sequence: dict[str, int] = {}
        self._next_status_time = 0.0
        self._alert_active = False
        self.vehicle_controller.reset()
        self.v2x.reset()
        self.safety_report: LaneSafetyReport = self._compute_safety_report()

    @property
    def all_vehicles(self) -> list[Vehicle]:
        """EGO followed by all traffic vehicles."""
        return [self.ego, *self.traffic]

    def toggle_pause(self) -> None:
        """Pause or resume time."""
        self.paused = not self.paused

    # ------------------------------------------------------------------- step
    def step(self, dt: float, control: Optional[ControlInput] = None) -> None:
        """Advance by ``dt`` seconds using fixed-size sub-steps (no-op when paused)."""
        if dt < 0:
            raise ValueError(f"dt must be >= 0, got {dt}")
        if self.paused or dt == 0:
            return
        control = control or ControlInput()
        remaining = min(dt, config.MAX_FRAME_DT_S)
        held_only = ControlInput(accelerate=control.accelerate, brake=control.brake)
        first = True
        hit: Optional[str] = None
        while remaining > 1e-9:
            h = min(remaining, config.PHYSICS_STEP_S)
            contact = self._substep(h, control if first else held_only)
            hit = hit or contact
            remaining -= h
            first = False
        self._update_collision(hit)
        self._recycle_traffic()
        self.safety_report = self._compute_safety_report()

    def _substep(self, h: float, control: ControlInput) -> Optional[str]:
        vehicles = self.all_vehicles
        decision = self.vehicle_controller.apply(self.ego, control, vehicles, h)
        level = self.vehicle_controller.last_avoidance.level
        if level.rank >= AvoidanceLevel.BRAKING.rank and not self._alert_active:
            self._alert_active = True
            self._send(make_emergency_brake_warning(self.ego, self.time, self._seq(self.ego.vehicle_id)))
        elif level.rank < AvoidanceLevel.BRAKING.rank:
            self._alert_active = False
        if decision is not None and decision.accepted and decision.target_lane is not None:
            self._send(make_lane_change_request(self.ego, decision.target_lane, self.time, self._seq(self.ego.vehicle_id)))

        for vehicle in self.traffic:
            vehicle.update_acceleration(vehicles)

        was_changing = self.ego.lane_change_in_progress
        for vehicle in vehicles:
            vehicle.update(h)
        self.time += h

        if was_changing and not self.ego.lane_change_in_progress:
            self._send(make_lane_change_complete(self.ego, self.time, self._seq(self.ego.vehicle_id)))
        while self.time >= self._next_status_time:
            self._send(make_status_broadcast(self.ego, self.time, self._seq(self.ego.vehicle_id)))
            self._next_status_time += config.STATUS_BROADCAST_PERIOD_S
        return self._resolve_overlaps()

    # ------------------------------------------------------------ V2X helpers
    def _seq(self, sender_id: str) -> int:
        value = self._sequence.get(sender_id, 0)
        self._sequence[sender_id] = value + 1
        return value

    def _send(self, message: V2XMessage) -> None:
        self.v2x.send(message, self.time)

    # --------------------------------------------------------------- bookkeeping
    def _resolve_overlaps(self) -> Optional[str]:
        """Hard no-overlap rule (simple bumper contact, not crash physics).

        If two bodies overlap, the vehicle further back is moved just behind the
        other and limited to its speed. Returns the ID of a vehicle the EGO
        touched during this sub-step, if any.
        """
        ego_contact: Optional[str] = None
        vehicles = self.all_vehicles
        for _ in range(2):  # second pass handles chains of three vehicles
            for i, a in enumerate(vehicles):
                for b in vehicles[i + 1:]:
                    if not a.overlaps(b):
                        continue
                    if a is self.ego:
                        ego_contact = b.vehicle_id
                    elif b is self.ego:
                        ego_contact = a.vehicle_id
                    rear, front = (a, b) if a.s <= b.s else (b, a)
                    rear.s = front.s - (front.length + rear.length) / 2.0 - config.CONTACT_MARGIN_M
                    rear.speed = min(rear.speed, front.speed)
        return ego_contact

    def _update_collision(self, hit: Optional[str]) -> None:
        if hit is not None and self.collision_with is None:
            self.collision_count += 1
        self.collision_with = hit

    def _compute_safety_report(self) -> LaneSafetyReport:
        return self.safety_checker.report(self.ego, self.all_vehicles, self.road)

    # ------------------------------------------------------------- traffic flow
    def _recycle_traffic(self) -> None:
        """Respawn traffic that fell far behind / ran far ahead, keeping density constant."""
        for vehicle in self.traffic:
            if vehicle.s < self.ego.s - config.TRAFFIC_RECYCLE_BEHIND_M:
                self._respawn(vehicle, config.SPAWN_AHEAD_RANGE_M)
            elif vehicle.s > self.ego.s + config.TRAFFIC_RECYCLE_AHEAD_M:
                self._respawn(vehicle, config.SPAWN_BEHIND_RANGE_M)

    def _respawn(self, vehicle: TrafficVehicle, offset_range: tuple[float, float]) -> None:
        for _ in range(config.SPAWN_ATTEMPTS):
            lane = self._rng.randrange(self.road.num_lanes)
            s = self.ego.s + self._rng.uniform(*offset_range)
            if self._spawn_is_free(vehicle, lane, s):
                speed = self._rng.uniform(*config.TRAFFIC_SPEED_RANGE_MPS)
                vehicle.place(lane, s, speed)
                vehicle.desired_speed = speed
                return
        # No free slot found: leave the vehicle where it is and retry next frame.

    def _spawn_is_free(self, vehicle: Vehicle, lane: int, s: float) -> bool:
        for other in self.all_vehicles:
            if other is vehicle:
                continue
            if lane in other.occupied_lanes() and abs(other.s - s) < config.SPAWN_MIN_GAP_M:
                return False
        return True

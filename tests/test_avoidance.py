"""Collision-avoidance assist: automatic slowing, evasive lane change, V2X warning."""
import math
import random

import pytest

import config
from communication import MessageType
from controllers import ControlInput, LaneChangeController, LaneDirection, VehicleController
from safety import AvoidanceLevel, CollisionAvoidance, SafetyChecker
from simulation.road import Road
from simulation.simulation import Simulation
from vehicles import EgoVehicle, TrafficVehicle


def traffic(vid, lane, s, speed):
    return TrafficVehicle(vid, lane=lane, s=s, speed=speed, desired_speed=speed)


def test_no_leader_means_clear():
    ego = EgoVehicle("EGO", lane=1, speed=25.0)
    cmd = CollisionAvoidance().assess(ego, [ego, traffic("T", 0, 10.0, 10.0)])
    assert cmd.level is AvoidanceLevel.CLEAR and cmd.decel == 0.0 and cmd.front_vehicle_id is None


def test_far_or_faster_leader_means_clear():
    ego = EgoVehicle("EGO", lane=1, speed=20.0)
    assert CollisionAvoidance().assess(ego, [ego, traffic("T", 1, 200.0, 10.0)]).level is AvoidanceLevel.CLEAR
    assert CollisionAvoidance().assess(ego, [ego, traffic("T", 1, 60.0, 30.0)]).level is AvoidanceLevel.CLEAR


def test_required_deceleration_formula():
    ego = EgoVehicle("EGO", lane=1, speed=20.0)
    leader = traffic("L", 1, 33.0 + config.VEHICLE_LENGTH_M, 10.0)   # bumper gap 33 m, closing 10 m/s
    cmd = CollisionAvoidance().assess(ego, [ego, leader])
    usable = 33.0 - config.AVOID_STOP_MARGIN_M
    assert cmd.required_decel == pytest.approx(100.0 / (2 * usable))
    assert cmd.decel == pytest.approx(cmd.required_decel * config.AVOID_BRAKE_FACTOR)
    assert cmd.ttc == pytest.approx(3.3)


def test_emergency_when_very_close_and_closing():
    ego = EgoVehicle("EGO", lane=1, speed=25.0)
    cmd = CollisionAvoidance().assess(ego, [ego, traffic("L", 1, 10.0, 5.0)])
    assert cmd.level is AvoidanceLevel.EMERGENCY_BRAKING and cmd.decel > 5.0 and cmd.wants_evasion


def test_gap_keeping_when_following_too_closely_at_same_speed():
    ego = EgoVehicle("EGO", lane=1, speed=20.0)
    cmd = CollisionAvoidance().assess(ego, [ego, traffic("L", 1, 12.0, 20.0)])
    assert cmd.level is AvoidanceLevel.SLOWING and cmd.decel > 0 and not cmd.wants_evasion


def test_avoidance_considers_vehicle_in_target_lane_during_lane_change():
    ego = EgoVehicle("EGO", lane=1, speed=25.0)
    ego.start_lane_change(0)
    cmd = CollisionAvoidance().assess(ego, [ego, traffic("L", 0, 15.0, 5.0)])
    assert cmd.front_vehicle_id == "L" and cmd.level is not AvoidanceLevel.CLEAR


def make_controller(auto_lane_change):
    road = Road()
    lcc = LaneChangeController(road, SafetyChecker(), enforce_safety=True)
    return VehicleController(lcc, CollisionAvoidance(), auto_lane_change=auto_lane_change)


def test_avoidance_overrides_driver_throttle():
    ctrl = make_controller(auto_lane_change=False)
    ego = EgoVehicle("EGO", lane=1, speed=25.0)
    leader = traffic("L", 1, 25.0, 5.0)
    ctrl.apply(ego, ControlInput(accelerate=True), [ego, leader], 0.02)
    assert ego.acceleration < 0
    assert "AUTO" in ctrl.last_action


def test_evasive_lane_change_when_neighbour_lane_is_free():
    ctrl = make_controller(auto_lane_change=True)
    ego = EgoVehicle("EGO", lane=1, speed=25.0)
    leader = traffic("L", 1, 30.0, 10.0)
    decision = ctrl.apply(ego, ControlInput(), [ego, leader], 0.02)
    assert decision is not None and decision.accepted
    assert ego.lane_change_in_progress and "evasive" in ctrl.last_action


def test_no_evasive_lane_change_when_toggle_off_or_lanes_blocked():
    ego = EgoVehicle("EGO", lane=1, speed=25.0)
    leader = traffic("L", 1, 30.0, 10.0)
    assert make_controller(False).apply(ego, ControlInput(), [ego, leader], 0.02) is None
    assert not ego.lane_change_in_progress and ego.acceleration < 0   # still brakes

    ego2 = EgoVehicle("EGO", lane=1, speed=25.0)
    blockers = [traffic("A", 0, 2.0, 25.0), traffic("B", 2, -2.0, 25.0)]
    assert make_controller(True).apply(ego2, ControlInput(), [ego2, leader, *blockers], 0.02) is None
    assert not ego2.lane_change_in_progress and ego2.acceleration < 0


def test_driver_holding_throttle_for_minutes_causes_no_collision():
    for auto in (True, False):
        sim = Simulation()
        sim.vehicle_controller.auto_lane_change = auto
        for _ in range(60 * 180):
            sim.step(1 / 60, ControlInput(accelerate=True))
        assert sim.collision_count == 0


def test_random_driving_with_sudden_braking_causes_no_collision():
    for seed in range(4):
        rng = random.Random(seed)
        sim = Simulation(seed=seed)
        for i in range(60 * 120):
            lane = rng.choice([None] * 40 + [LaneDirection.LEFT, LaneDirection.RIGHT])
            sim.step(1 / 60, ControlInput(rng.random() < 0.7, rng.random() < 0.1, lane))
            if i % 1200 == 600:
                t = rng.choice(sim.traffic)
                t.desired_speed = max(5.0, t.speed - 12.0)
        assert sim.collision_count == 0, f"seed {seed}"


def test_hard_braking_creates_one_v2x_warning_message():
    sim = Simulation()
    sim.traffic[1].place(sim.ego.lane, sim.ego.s + 20.0, 5.0)   # sudden slow car close ahead
    sim.vehicle_controller.auto_lane_change = False
    for _ in range(60 * 5):
        sim.step(1 / 60, ControlInput(accelerate=True))
    warnings = [e for e in sim.v2x.recent_events(50) if e.message.message_type is MessageType.EMERGENCY_BRAKE_WARNING]
    assert len(warnings) >= 1
    assert sim.collision_count == 0


def test_feed_forward_reacts_to_braking_leader_before_gap_closes():
    ego = EgoVehicle("EGO", lane=1, speed=20.0)
    leader = traffic("L", 1, 25.0 + config.VEHICLE_LENGTH_M, 20.0)    # same speed, 25 m gap
    leader.acceleration = -8.0
    cmd = CollisionAvoidance().assess(ego, [ego, leader])
    assert cmd.closing_speed == pytest.approx(0.0)
    assert cmd.decel >= 7.0 and cmd.level is AvoidanceLevel.EMERGENCY_BRAKING


def test_far_braking_leader_does_not_trigger_feed_forward():
    ego = EgoVehicle("EGO", lane=1, speed=20.0)
    leader = traffic("L", 1, 120.0, 20.0)
    leader.acceleration = -8.0
    assert CollisionAvoidance().assess(ego, [ego, leader]).level is AvoidanceLevel.CLEAR

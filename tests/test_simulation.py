"""Headless tests of the simulation loop (no pygame window needed)."""
import pytest

import config
from controllers import ControlInput, LaneDirection
from communication import MessageType
from simulation.simulation import Simulation


def run(sim, seconds, control=None, dt=1 / 60):
    for _ in range(int(seconds / dt)):
        sim.step(dt, control)


def test_initial_scenario():
    sim = Simulation()
    assert sim.ego.vehicle_id == "EGO" and len(sim.traffic) == len(config.INITIAL_TRAFFIC)
    assert len({v.vehicle_id for v in sim.all_vehicles}) == len(sim.all_vehicles)


def test_pause_freezes_time_and_reset_restores():
    sim = Simulation()
    run(sim, 1.0)
    t = sim.time
    sim.toggle_pause()
    run(sim, 1.0)
    assert sim.time == t
    sim.toggle_pause()
    sim.reset()
    assert sim.time == 0.0 and sim.ego.s == config.EGO_INITIAL_S and not sim.paused


def test_long_run_keeps_everyone_on_the_road_and_within_speed_limits():
    sim = Simulation()
    run(sim, 60.0, ControlInput(accelerate=True))
    for v in sim.all_vehicles:
        assert 0 <= v.lane < sim.road.num_lanes
        assert v.width / 2 <= v.lateral <= sim.road.width - v.width / 2
        assert v.min_speed <= v.speed <= v.max_speed
    assert sim.ego.speed <= config.EGO_MAX_SPEED_MPS


def test_traffic_is_recycled_near_ego():
    sim = Simulation()
    run(sim, 60.0, ControlInput(accelerate=True))
    for v in sim.traffic:
        assert -config.TRAFFIC_RECYCLE_BEHIND_M - 1 <= v.s - sim.ego.s <= config.TRAFFIC_RECYCLE_AHEAD_M + 1


def test_same_seed_is_deterministic():
    a, b = Simulation(seed=1), Simulation(seed=1)
    run(a, 20.0, ControlInput(accelerate=True))
    run(b, 20.0, ControlInput(accelerate=True))
    assert [round(v.s, 6) for v in a.all_vehicles] == [round(v.s, 6) for v in b.all_vehicles]


def test_lane_request_and_completion_flow_with_logging():
    sim = Simulation(enforce_safety=False)
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.LEFT))
    assert sim.ego.target_lane == 0
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.RIGHT))   # ignored: in progress
    assert "rejected" in sim.vehicle_controller.last_action
    run(sim, config.LANE_CHANGE_DURATION_S + 0.5)
    assert sim.ego.lane == 0 and sim.ego.target_lane is None
    types = [e.message.message_type for e in sim.v2x.recent_events(50)]
    assert MessageType.LANE_CHANGE_REQUEST in types and MessageType.LANE_CHANGE_COMPLETE in types
    assert MessageType.VEHICLE_STATUS in types


def test_lane_request_at_boundary_is_rejected_without_crash():
    sim = Simulation(enforce_safety=False)
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.LEFT))
    run(sim, config.LANE_CHANGE_DURATION_S + 0.2)
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.LEFT))
    assert "rejected" in sim.vehicle_controller.last_action and sim.ego.lane == 0


def test_frame_rate_independence_of_whole_simulation():
    fast, slow = Simulation(), Simulation()
    run(fast, 5.0, ControlInput(accelerate=True), dt=1 / 120)
    run(slow, 5.0, ControlInput(accelerate=True), dt=1 / 30)
    assert fast.ego.s == pytest.approx(slow.ego.s, rel=1e-3)


def test_collision_is_detected_and_counted():
    sim = Simulation()
    sim.traffic[0].place(sim.ego.lane, sim.ego.s + 1.0, 20.0)
    sim.step(1 / 60)
    assert sim.collision_with == sim.traffic[0].vehicle_id and sim.collision_count == 1


def test_negative_dt_raises():
    with pytest.raises(ValueError):
        Simulation().step(-1.0)


def assert_no_overlaps(sim):
    vehicles = sim.all_vehicles
    for i, a in enumerate(vehicles):
        for b in vehicles[i + 1:]:
            assert not a.overlaps(b), f"{a.vehicle_id} overlaps {b.vehicle_id} at t={sim.time:.2f}"


def test_unsafe_lane_change_is_blocked_by_default():
    sim = Simulation()   # initial scenario: T4 closing fast in lane 0 -> not SAFE
    sim.step(1 / 60, ControlInput(lane_request=LaneDirection.LEFT))
    assert sim.ego.target_lane is None
    assert "rejected" in sim.vehicle_controller.last_action


def test_ego_brakes_behind_slow_leader_instead_of_hitting_it():
    sim = Simulation()
    sim.vehicle_controller.auto_lane_change = False   # force it to stay behind
    leader = sim.traffic[1]                           # T2: slow car in EGO's lane
    for _ in range(60 * 40):
        sim.step(1 / 60, ControlInput(accelerate=True))
        assert_no_overlaps(sim)
    assert sim.ego.s < leader.s                       # still behind the leader
    assert sim.collision_count == 0                   # and never touched it
    assert sim.ego.speed == pytest.approx(leader.speed, abs=1.5)


def test_ego_overtakes_slow_leader_by_evasive_lane_change():
    sim = Simulation()
    assert sim.vehicle_controller.auto_lane_change
    for _ in range(60 * 60):
        sim.step(1 / 60, ControlInput(accelerate=True))
    assert sim.collision_count == 0
    assert sim.ego.s > sim.traffic[1].s or sim.ego.lane != 1


def test_no_vehicle_pair_ever_overlaps_under_random_driving():
    import random
    rng = random.Random(5)
    sim = Simulation(enforce_safety=False)        # worst case: unsafe changes allowed
    for _ in range(60 * 120):
        lane = rng.choice([None] * 30 + [LaneDirection.LEFT, LaneDirection.RIGHT])
        sim.step(1 / 60, ControlInput(rng.random() < 0.6, rng.random() < 0.2, lane))
        assert_no_overlaps(sim)

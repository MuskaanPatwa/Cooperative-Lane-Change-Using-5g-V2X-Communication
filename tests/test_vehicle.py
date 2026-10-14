"""Vehicle initialisation, speed limits, acceleration, boundaries."""
import math

import pytest

import config
from simulation.road import Road
from vehicles import EgoVehicle, ManeuverState, TrafficVehicle, Vehicle


def test_vehicle_initialization_defaults():
    v = Vehicle("V1", lane=1, s=10.0, speed=15.0)
    assert v.vehicle_id == "V1"
    assert v.lane == 1 and v.target_lane is None
    assert v.maneuver is ManeuverState.CRUISING
    assert v.length == config.VEHICLE_LENGTH_M and v.width == config.VEHICLE_WIDTH_M
    assert v.lateral == pytest.approx(Road().lane_center(1))
    assert v.acceleration == 0.0
    assert v.occupied_lanes() == (1,)


@pytest.mark.parametrize("lane", [-1, 3, 99])
def test_vehicle_rejects_lane_outside_road(lane):
    with pytest.raises(ValueError, match="outside the road"):
        Vehicle("V", lane=lane)


def test_vehicle_rejects_invalid_parameters():
    with pytest.raises(ValueError):
        Vehicle("", lane=0)
    with pytest.raises(ValueError):
        Vehicle("V", lane=0, length=0)
    with pytest.raises(ValueError):
        Vehicle("V", lane=0, min_speed=10, max_speed=5)


def test_ego_defaults_and_speed_clamped_on_init():
    ego = EgoVehicle("EGO", lane=1, speed=1000.0)
    assert ego.speed == config.EGO_MAX_SPEED_MPS
    ego2 = EgoVehicle("EGO", lane=1, speed=0.0)
    assert ego2.speed == config.EGO_MIN_SPEED_MPS


def test_position_integrates_with_constant_speed():
    v = Vehicle("V", lane=0, s=0.0, speed=10.0)
    v.update(2.0)
    assert v.s == pytest.approx(20.0)


def test_acceleration_changes_speed_and_distance():
    v = Vehicle("V", lane=0, s=0.0, speed=10.0)
    v.set_acceleration(2.0)
    v.update(1.0)
    assert v.speed == pytest.approx(12.0)
    assert v.s == pytest.approx(11.0)  # average speed 11 m/s over 1 s


def test_ego_speed_never_exceeds_limits():
    ego = EgoVehicle("EGO", lane=1, speed=20.0)
    ego.command(accelerate=True, brake=False)
    for _ in range(2000):
        ego.update(0.05)
    assert ego.speed == pytest.approx(config.EGO_MAX_SPEED_MPS)
    ego.command(accelerate=False, brake=True)
    for _ in range(2000):
        ego.update(0.05)
    assert ego.speed == pytest.approx(config.EGO_MIN_SPEED_MPS)


def test_ego_command_priorities():
    ego = EgoVehicle("EGO", lane=1)
    ego.command(True, True)
    assert ego.acceleration < 0           # brake wins
    ego.command(False, False)
    assert ego.acceleration == 0.0        # coast


def test_acceleration_is_limited_to_physical_limits():
    v = Vehicle("V", lane=0)
    v.set_acceleration(100.0)
    assert v.acceleration == config.MAX_ACCEL_MPS2
    v.set_acceleration(-100.0)
    assert v.acceleration == -config.MAX_DECEL_MPS2
    with pytest.raises(ValueError):
        v.set_acceleration(math.nan)


def test_update_rejects_negative_or_invalid_dt():
    v = Vehicle("V", lane=0)
    with pytest.raises(ValueError):
        v.update(-0.1)
    with pytest.raises(ValueError):
        v.update(math.inf)
    s = v.s
    v.update(0.0)
    assert v.s == s


def test_frame_rate_independence_of_longitudinal_motion():
    a = Vehicle("A", lane=0, speed=10.0)
    b = Vehicle("B", lane=0, speed=10.0)
    a.set_acceleration(1.5)
    b.set_acceleration(1.5)
    for _ in range(100):
        a.update(0.01)
    for _ in range(10):
        b.update(0.1)
    assert a.s == pytest.approx(b.s, rel=1e-6)
    assert a.speed == pytest.approx(b.speed)


def test_overlap_detection():
    a = Vehicle("A", lane=0, s=0.0)
    b = Vehicle("B", lane=0, s=3.0)
    c = Vehicle("C", lane=1, s=0.0)
    d = Vehicle("D", lane=0, s=10.0)
    assert a.overlaps(b)
    assert not a.overlaps(c)
    assert not a.overlaps(d)


def test_traffic_vehicle_holds_desired_speed_when_free():
    t = TrafficVehicle("T", lane=0, speed=10.0, desired_speed=20.0)
    for _ in range(2000):
        t.update_acceleration([t])
        t.update(0.05)
    assert t.speed == pytest.approx(20.0, abs=0.1)


def test_traffic_vehicle_brakes_behind_slow_leader():
    leader = TrafficVehicle("L", lane=0, s=40.0, speed=10.0, desired_speed=10.0)
    follower = TrafficVehicle("F", lane=0, s=0.0, speed=25.0, desired_speed=25.0)
    min_gap = math.inf
    for _ in range(4000):
        for v in (leader, follower):
            v.update_acceleration([leader, follower])
            v.update(0.02)
        min_gap = min(min_gap, leader.s - follower.s - config.VEHICLE_LENGTH_M)
    assert min_gap > 0.0
    assert follower.speed == pytest.approx(10.0, abs=0.5)

"""Gap, relative-speed and TTC calculations plus the SAFE/UNSAFE/WAITING outcomes."""
import math

import pytest

from safety import SafetyChecker, SafetyConfig, SafetyStatus, bumper_gap, compute_lane_gaps, time_to_collision
from simulation.road import Road
from vehicles import Vehicle

ROAD = Road()


def veh(vid, lane, s, speed=20.0):
    return Vehicle(vid, lane=lane, s=s, speed=speed)


def test_bumper_gap_uses_vehicle_lengths():
    rear, front = veh("R", 0, 0.0), veh("F", 0, 20.0)
    assert bumper_gap(rear, front) == pytest.approx(20.0 - 4.5)
    assert bumper_gap(rear, veh("O", 0, 3.0)) < 0       # overlapping bodies


def test_front_and_rear_gap_calculation():
    ego = veh("EGO", 1, 100.0)
    vehicles = [ego, veh("A", 1, 130.0), veh("B", 1, 150.0), veh("C", 1, 80.0), veh("D", 1, 40.0), veh("X", 2, 105.0)]
    gaps = compute_lane_gaps(ego, vehicles, 1)
    assert gaps.front_vehicle_id == "A" and gaps.front_gap == pytest.approx(30.0 - 4.5)
    assert gaps.rear_vehicle_id == "C" and gaps.rear_gap == pytest.approx(20.0 - 4.5)
    assert gaps.overlapping_vehicle_id is None


def test_no_neighbours_gives_infinite_gaps():
    ego = veh("EGO", 1, 0.0)
    gaps = compute_lane_gaps(ego, [ego, veh("X", 2, 5.0)], 1)
    assert math.isinf(gaps.front_gap) and math.isinf(gaps.rear_gap)
    assert math.isinf(gaps.front_ttc) and math.isinf(gaps.rear_ttc)
    assert gaps.front_vehicle_id is None and gaps.rear_vehicle_id is None


def test_vehicle_changing_lane_counts_for_both_lanes():
    ego = veh("EGO", 1, 0.0)
    other = veh("O", 0, 30.0)
    other.start_lane_change(1)
    assert compute_lane_gaps(ego, [ego, other], 1).front_vehicle_id == "O"
    assert compute_lane_gaps(ego, [ego, other], 0).front_vehicle_id == "O"


def test_ttc_for_approaching_vehicles():
    assert time_to_collision(30.0, 10.0) == pytest.approx(3.0)
    assert time_to_collision(10.0, 2.0) == pytest.approx(5.0)


def test_ttc_is_infinite_when_not_closing():
    assert math.isinf(time_to_collision(30.0, 0.0))
    assert math.isinf(time_to_collision(30.0, -5.0))
    assert time_to_collision(0.0, 5.0) == 0.0


def test_ttc_front_and_rear_closing_speeds():
    ego = veh("EGO", 1, 0.0, speed=25.0)
    front = veh("F", 1, 54.5, speed=20.0)    # gap 50 m, closing 5 m/s -> 10 s
    rear = veh("R", 1, -34.5, speed=30.0)    # gap 30 m, closing 5 m/s -> 6 s
    gaps = compute_lane_gaps(ego, [ego, front, rear], 1)
    assert gaps.front_closing_speed == pytest.approx(5.0)
    assert gaps.front_ttc == pytest.approx(10.0)
    assert gaps.rear_closing_speed == pytest.approx(5.0)
    assert gaps.rear_ttc == pytest.approx(6.0)


def test_slower_rear_and_faster_front_do_not_close():
    ego = veh("EGO", 1, 0.0, speed=20.0)
    gaps = compute_lane_gaps(ego, [ego, veh("F", 1, 30.0, 25.0), veh("R", 1, -30.0, 15.0)], 1)
    assert math.isinf(gaps.front_ttc) and math.isinf(gaps.rear_ttc)


def test_safe_when_gaps_large():
    ego = veh("EGO", 1, 0.0)
    result = SafetyChecker().evaluate_lane_change(ego, 0, [ego, veh("F", 0, 60.0), veh("R", 0, -60.0)], ROAD)
    assert result.status is SafetyStatus.SAFE and not result.reasons


def test_unsafe_when_target_slot_occupied():
    ego = veh("EGO", 1, 0.0)
    result = SafetyChecker().evaluate_lane_change(ego, 0, [ego, veh("B", 0, 2.0)], ROAD)
    assert result.status is SafetyStatus.UNSAFE
    assert any("occupied" in r for r in result.reasons)


def test_unsafe_when_ttc_too_low():
    ego = veh("EGO", 1, 0.0, speed=30.0)
    front = veh("F", 0, 25.0, speed=20.0)    # gap 20.5 m, closing 10 -> TTC 2.05 s < 3 s
    result = SafetyChecker().evaluate_lane_change(ego, 0, [ego, front], ROAD)
    assert result.status is SafetyStatus.UNSAFE
    assert any("TTC" in r for r in result.reasons)


def test_waiting_for_gap_when_small_but_not_closing():
    ego = veh("EGO", 1, 0.0, speed=20.0)
    front = veh("F", 0, 10.0, speed=20.0)    # gap 5.5 m, same speed
    result = SafetyChecker().evaluate_lane_change(ego, 0, [ego, front], ROAD)
    assert result.status is SafetyStatus.WAITING_FOR_GAP


def test_custom_thresholds_are_respected():
    ego = veh("EGO", 1, 0.0)
    front = veh("F", 0, 20.0)                # gap 15.5 m
    strict = SafetyChecker(SafetyConfig(min_front_gap_m=30.0))
    assert strict.evaluate_lane_change(ego, 0, [ego, front], ROAD).status is SafetyStatus.WAITING_FOR_GAP
    assert SafetyChecker().evaluate_lane_change(ego, 0, [ego, front], ROAD).status is SafetyStatus.SAFE


def test_road_boundary_check():
    ego = veh("EGO", 0, 0.0)
    result = SafetyChecker().evaluate_lane_change(ego, -1, [ego], ROAD)
    assert result.status is SafetyStatus.UNSAFE and result.out_of_road
    assert "N/A" in result.summary()


def test_report_covers_both_sides_and_own_lane():
    ego = veh("EGO", 0, 0.0)
    report = SafetyChecker().report(ego, [ego, veh("F", 0, 40.0)], ROAD)
    assert report.left.out_of_road
    assert report.right.status is SafetyStatus.SAFE
    assert report.own_lane.front_vehicle_id == "F"

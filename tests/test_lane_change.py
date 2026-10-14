"""Lane-change validation, smooth transitions and road boundaries."""
import pytest

import config
from controllers import LaneChangeController, LaneDirection
from safety import SafetyChecker
from simulation.road import Road
from vehicles import EgoVehicle, LaneChangeError, ManeuverState, TrafficVehicle, Vehicle


@pytest.fixture
def road():
    return Road()


@pytest.fixture
def controller(road):
    return LaneChangeController(road, SafetyChecker(), enforce_safety=False)


def test_road_boundaries(road):
    assert road.is_valid_lane(0) and road.is_valid_lane(2)
    assert not road.is_valid_lane(-1) and not road.is_valid_lane(3)
    assert not road.is_valid_lane(True) and not road.is_valid_lane("1")
    assert road.width == pytest.approx(3 * config.LANE_WIDTH_M)
    with pytest.raises(ValueError):
        road.lane_center(5)
    assert road.clamp_lateral(-5.0, 1.9) == pytest.approx(0.95)
    assert road.clamp_lateral(99.0, 1.9) == pytest.approx(road.width - 0.95)
    assert road.lane_at(-3.0) == 0 and road.lane_at(99.0) == 2


def test_valid_left_and_right_requests(controller):
    ego = EgoVehicle("EGO", lane=1)
    decision = controller.request_lane_change(ego, LaneDirection.LEFT, [ego])
    assert decision.accepted and decision.target_lane == 0
    assert ego.target_lane == 0 and ego.maneuver is ManeuverState.LANE_CHANGE_LEFT

    ego2 = EgoVehicle("EGO", lane=1)
    decision = controller.request_lane_change(ego2, LaneDirection.RIGHT, [ego2])
    assert decision.accepted and ego2.maneuver is ManeuverState.LANE_CHANGE_RIGHT


@pytest.mark.parametrize("lane,direction", [(0, LaneDirection.LEFT), (2, LaneDirection.RIGHT)])
def test_requests_beyond_road_boundary_are_rejected(controller, lane, direction):
    ego = EgoVehicle("EGO", lane=lane)
    decision = controller.request_lane_change(ego, direction, [ego])
    assert not decision.accepted
    assert "road boundary" in decision.reason
    assert ego.target_lane is None and ego.maneuver is ManeuverState.CRUISING


def test_request_rejected_while_transition_in_progress(controller):
    ego = EgoVehicle("EGO", lane=1)
    assert controller.request_lane_change(ego, LaneDirection.LEFT, [ego]).accepted
    ego.update(0.5)
    second = controller.request_lane_change(ego, LaneDirection.RIGHT, [ego])
    assert not second.accepted and "in progress" in second.reason
    assert ego.target_lane == 0


def test_invalid_direction_is_handled_gracefully(controller):
    ego = EgoVehicle("EGO", lane=1)
    decision = controller.request_lane_change(ego, "up", [ego])
    assert not decision.accepted and "invalid direction" in decision.reason


def test_vehicle_start_lane_change_errors():
    v = Vehicle("V", lane=0)
    with pytest.raises(LaneChangeError):
        v.start_lane_change(-1)
    with pytest.raises(LaneChangeError):
        v.start_lane_change(0)       # same lane
    with pytest.raises(LaneChangeError):
        v.start_lane_change(2)       # not adjacent
    v.start_lane_change(1)
    with pytest.raises(LaneChangeError):
        v.start_lane_change(0)       # already changing


def test_lane_change_is_smooth_and_completes(road):
    ego = EgoVehicle("EGO", lane=1)
    ego.start_lane_change(2)
    start, end = road.lane_center(1), road.lane_center(2)
    previous = ego.lateral
    max_jump = 0.0
    steps = int(config.LANE_CHANGE_DURATION_S / 0.02)
    for i in range(steps):
        ego.update(0.02)
        if i < steps - 1:
            assert ego.lane == 1 and ego.target_lane == 2     # still in transition
            assert start <= ego.lateral <= end
        assert ego.lateral >= previous - 1e-9                   # monotonic
        max_jump = max(max_jump, abs(ego.lateral - previous))
        previous = ego.lateral
    assert max_jump < 0.2                                       # no teleporting (lane = 3.7 m)
    assert ego.lane == 2 and ego.target_lane is None
    assert ego.maneuver is ManeuverState.CRUISING
    assert ego.lateral == pytest.approx(end)
    assert ego.lateral_speed == 0.0


def test_lane_change_midpoint_is_halfway_and_progress_reported():
    ego = EgoVehicle("EGO", lane=1)
    ego.start_lane_change(0)
    ego.update(config.LANE_CHANGE_DURATION_S / 2)
    assert ego.lane_change_progress == pytest.approx(0.5)
    assert ego.lateral == pytest.approx((Road().lane_center(1) + Road().lane_center(0)) / 2)
    assert ego.occupied_lanes() == (1, 0)


def test_lane_change_is_frame_rate_independent():
    a, b = EgoVehicle("A", lane=1), EgoVehicle("B", lane=1)
    a.start_lane_change(0)
    b.start_lane_change(0)
    for _ in range(50):
        a.update(0.01)           # 0.5 s
    for _ in range(5):
        b.update(0.1)            # 0.5 s
    assert a.lateral == pytest.approx(b.lateral)


def test_vehicle_never_leaves_road_during_transition(road):
    v = Vehicle("V", lane=0)
    v.start_lane_change(1)
    for _ in range(300):
        v.update(0.02)
        assert v.width / 2 <= v.lateral <= road.width - v.width / 2


def test_enforced_safety_blocks_unsafe_change(road):
    controller = LaneChangeController(road, SafetyChecker(), enforce_safety=True)
    ego = EgoVehicle("EGO", lane=1, s=0.0)
    blocker = TrafficVehicle("T", lane=0, s=1.0, speed=20.0)
    decision = controller.request_lane_change(ego, LaneDirection.LEFT, [ego, blocker])
    assert not decision.accepted and "UNSAFE" in decision.reason
    assert ego.target_lane is None


def test_informative_safety_does_not_block_in_phase1(controller):
    ego = EgoVehicle("EGO", lane=1, s=0.0)
    blocker = TrafficVehicle("T", lane=0, s=1.0, speed=20.0)
    decision = controller.request_lane_change(ego, LaneDirection.LEFT, [ego, blocker])
    assert decision.accepted
    assert "informative only" in decision.reason

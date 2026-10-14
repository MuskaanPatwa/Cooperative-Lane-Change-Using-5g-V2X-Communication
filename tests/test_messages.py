"""V2X message creation, serialisation and validation."""
import json
import math

import pytest

from communication import (
    Maneuver, MessageType, MessageValidationError, NullV2XChannel, V2XMessage,
    make_lane_change_complete, make_lane_change_request, make_lane_change_response,
    make_status_broadcast, make_emergency_brake_warning,
)
from vehicles import Vehicle


def valid_kwargs(**overrides):
    base = dict(sender_id="EGO", message_type=MessageType.VEHICLE_STATUS, timestamp=1.5,
                position=100.0, speed=20.0, lane=1)
    base.update(overrides)
    return base


def test_message_creation_with_defaults():
    msg = V2XMessage(**valid_kwargs())
    assert msg.target_lane is None
    assert msg.intended_maneuver is Maneuver.NONE
    assert msg.sequence_number is None and msg.recipient_id is None


def test_message_is_immutable():
    msg = V2XMessage(**valid_kwargs())
    with pytest.raises(Exception):
        msg.speed = 5.0  # type: ignore[misc]


def test_dict_and_json_round_trip():
    msg = V2XMessage(**valid_kwargs(message_type=MessageType.LANE_CHANGE_REQUEST, target_lane=0,
                                    intended_maneuver=Maneuver.LANE_CHANGE_LEFT, sequence_number=7))
    assert V2XMessage.from_dict(msg.to_dict()) == msg
    text = msg.to_json()
    assert json.loads(text)["message_type"] == "LANE_CHANGE_REQUEST"
    assert V2XMessage.from_json(text) == msg


@pytest.mark.parametrize("field", ["sender_id", "message_type", "timestamp", "position", "speed", "lane"])
def test_missing_required_field_raises(field):
    data = V2XMessage(**valid_kwargs()).to_dict()
    del data[field]
    with pytest.raises(MessageValidationError, match=field):
        V2XMessage.from_dict(data)


def test_none_required_field_counts_as_missing():
    data = V2XMessage(**valid_kwargs()).to_dict()
    data["speed"] = None
    with pytest.raises(MessageValidationError, match="speed"):
        V2XMessage.from_dict(data)


@pytest.mark.parametrize("overrides", [
    dict(sender_id=""), dict(sender_id=5), dict(message_type="VEHICLE_STATUS"),
    dict(timestamp=-1.0), dict(timestamp=math.nan), dict(speed=-3.0), dict(speed="fast"),
    dict(position=math.inf), dict(lane=-1), dict(lane=1.5), dict(lane=True),
    dict(target_lane=-2), dict(sequence_number=-1), dict(intended_maneuver="LEFT"),
    dict(recipient_id=""),
])
def test_invalid_values_raise(overrides):
    with pytest.raises(MessageValidationError):
        V2XMessage(**valid_kwargs(**overrides))


def test_lane_change_request_requires_target_lane():
    with pytest.raises(MessageValidationError, match="target_lane"):
        V2XMessage(**valid_kwargs(message_type=MessageType.LANE_CHANGE_REQUEST))


def test_from_dict_rejects_unknown_enums_and_wrong_types():
    data = V2XMessage(**valid_kwargs()).to_dict()
    with pytest.raises(MessageValidationError, match="unknown message_type"):
        V2XMessage.from_dict({**data, "message_type": "HELLO"})
    with pytest.raises(MessageValidationError, match="unknown intended_maneuver"):
        V2XMessage.from_dict({**data, "intended_maneuver": "FLY"})
    with pytest.raises(MessageValidationError):
        V2XMessage.from_dict([1, 2, 3])  # type: ignore[arg-type]


@pytest.mark.parametrize("text", ["not json", "", "[1, 2]", None])
def test_from_json_rejects_bad_input(text):
    with pytest.raises(MessageValidationError):
        V2XMessage.from_json(text)  # type: ignore[arg-type]


def test_factories_fill_fields_from_vehicle():
    v = Vehicle("EGO", lane=1, s=50.0, speed=22.0)
    status = make_status_broadcast(v, 2.0, 3)
    assert (status.message_type, status.lane, status.position, status.sequence_number) == (MessageType.VEHICLE_STATUS, 1, 50.0, 3)
    assert status.intended_maneuver is Maneuver.KEEP_LANE

    request = make_lane_change_request(v, 0, 2.0)
    assert request.message_type is MessageType.LANE_CHANGE_REQUEST
    assert request.target_lane == 0 and request.intended_maneuver is Maneuver.LANE_CHANGE_LEFT

    assert make_lane_change_request(v, 2, 2.0).intended_maneuver is Maneuver.LANE_CHANGE_RIGHT

    ack = make_lane_change_response(v, True, 0, 2.1, recipient_id="T1")
    rej = make_lane_change_response(v, False, 0, 2.1, recipient_id="T1")
    assert ack.message_type is MessageType.LANE_CHANGE_ACK
    assert rej.message_type is MessageType.LANE_CHANGE_REJECT
    assert ack.recipient_id == "T1"

    done = make_lane_change_complete(v, 5.0)
    assert done.message_type is MessageType.LANE_CHANGE_COMPLETE and done.target_lane is None


def test_status_during_lane_change_reports_intention():
    v = Vehicle("EGO", lane=1)
    v.start_lane_change(2)
    status = make_status_broadcast(v, 1.0)
    assert status.target_lane == 2 and status.intended_maneuver is Maneuver.LANE_CHANGE_RIGHT


def test_null_channel_logs_but_never_delivers():
    channel = NullV2XChannel()
    msg = V2XMessage(**valid_kwargs(sequence_number=1))
    channel.send(msg, 1.0)
    assert channel.sent_count == 1
    assert channel.receive("T1", 2.0) == []
    events = channel.recent_events()
    assert len(events) == 1 and "NOT DELIVERED" in events[0].describe()
    channel.reset()
    assert channel.recent_events() == [] and channel.sent_count == 0


def test_emergency_brake_warning_factory():
    v = Vehicle("EGO", lane=1, s=10.0, speed=18.0)
    msg = make_emergency_brake_warning(v, 4.0, 9)
    assert msg.message_type is MessageType.EMERGENCY_BRAKE_WARNING
    assert V2XMessage.from_json(msg.to_json()) == msg

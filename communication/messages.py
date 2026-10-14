"""V2X-style message data structures (Phase 1: definitions only).

These messages are plain Python objects with validation and JSON
(de)serialisation. They are a simplified research model, NOT the encoding of
any standard (3GPP NR-V2X, ETSI CAM/DENM, SAE J2735 BSM, ...).

Lane indices are the simulation's 0-based indices (0 = leftmost lane).
``recipient_id`` is an optional extension (``None`` = broadcast) so that
acknowledgements/rejections can name the vehicle they answer.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any, Mapping, Optional

if TYPE_CHECKING:
    from vehicles.vehicle import Vehicle


class MessageType(str, Enum):
    """Supported message types."""

    VEHICLE_STATUS = "VEHICLE_STATUS"
    LANE_CHANGE_REQUEST = "LANE_CHANGE_REQUEST"
    LANE_CHANGE_ACK = "LANE_CHANGE_ACK"
    LANE_CHANGE_REJECT = "LANE_CHANGE_REJECT"
    LANE_CHANGE_COMPLETE = "LANE_CHANGE_COMPLETE"
    EMERGENCY_BRAKE_WARNING = "EMERGENCY_BRAKE_WARNING"   # extension: hard braking started


class Maneuver(str, Enum):
    """Intended maneuver carried inside a message."""

    NONE = "NONE"
    KEEP_LANE = "KEEP_LANE"
    LANE_CHANGE_LEFT = "LANE_CHANGE_LEFT"
    LANE_CHANGE_RIGHT = "LANE_CHANGE_RIGHT"


class MessageValidationError(ValueError):
    """Raised for missing, malformed or invalid message fields."""


REQUIRED_FIELDS = ("sender_id", "message_type", "timestamp", "position", "speed", "lane")


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _is_index(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


@dataclass(frozen=True)
class V2XMessage:
    """One V2X-style message. Validated on construction."""

    sender_id: str
    message_type: MessageType
    timestamp: float
    position: float
    speed: float
    lane: int
    target_lane: Optional[int] = None
    intended_maneuver: Maneuver = Maneuver.NONE
    sequence_number: Optional[int] = None
    recipient_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.sender_id, str) or not self.sender_id.strip():
            raise MessageValidationError("sender_id must be a non-empty string")
        if not isinstance(self.message_type, MessageType):
            raise MessageValidationError(f"message_type must be a MessageType, got {self.message_type!r}")
        if not _is_number(self.timestamp) or self.timestamp < 0:
            raise MessageValidationError(f"timestamp must be a finite number >= 0, got {self.timestamp!r}")
        if not _is_number(self.position):
            raise MessageValidationError(f"position must be a finite number, got {self.position!r}")
        if not _is_number(self.speed) or self.speed < 0:
            raise MessageValidationError(f"speed must be a finite number >= 0, got {self.speed!r}")
        if not _is_index(self.lane):
            raise MessageValidationError(f"lane must be an integer >= 0, got {self.lane!r}")
        if self.target_lane is not None and not _is_index(self.target_lane):
            raise MessageValidationError(f"target_lane must be None or an integer >= 0, got {self.target_lane!r}")
        if not isinstance(self.intended_maneuver, Maneuver):
            raise MessageValidationError(f"intended_maneuver must be a Maneuver, got {self.intended_maneuver!r}")
        if self.sequence_number is not None and not _is_index(self.sequence_number):
            raise MessageValidationError(f"sequence_number must be None or an integer >= 0, got {self.sequence_number!r}")
        if self.recipient_id is not None and (not isinstance(self.recipient_id, str) or not self.recipient_id.strip()):
            raise MessageValidationError("recipient_id must be None or a non-empty string")
        if self.message_type is MessageType.LANE_CHANGE_REQUEST and self.target_lane is None:
            raise MessageValidationError("LANE_CHANGE_REQUEST requires target_lane")

    # ----------------------------------------------------------- serialisation
    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dictionary (enums become strings)."""
        return {
            "sender_id": self.sender_id,
            "message_type": self.message_type.value,
            "timestamp": self.timestamp,
            "position": self.position,
            "speed": self.speed,
            "lane": self.lane,
            "target_lane": self.target_lane,
            "intended_maneuver": self.intended_maneuver.value,
            "sequence_number": self.sequence_number,
            "recipient_id": self.recipient_id,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "V2XMessage":
        """Build a message from a dictionary, raising :class:`MessageValidationError` on problems."""
        if not isinstance(data, Mapping):
            raise MessageValidationError(f"message data must be a mapping, got {type(data).__name__}")
        missing = [name for name in REQUIRED_FIELDS if data.get(name) is None]
        if missing:
            raise MessageValidationError(f"missing required field(s): {', '.join(missing)}")
        try:
            message_type = MessageType(data["message_type"])
        except ValueError:
            raise MessageValidationError(f"unknown message_type {data['message_type']!r}") from None
        try:
            maneuver = Maneuver(data.get("intended_maneuver") or Maneuver.NONE.value)
        except ValueError:
            raise MessageValidationError(f"unknown intended_maneuver {data.get('intended_maneuver')!r}") from None
        return cls(
            sender_id=data["sender_id"],
            message_type=message_type,
            timestamp=data["timestamp"],
            position=data["position"],
            speed=data["speed"],
            lane=data["lane"],
            target_lane=data.get("target_lane"),
            intended_maneuver=maneuver,
            sequence_number=data.get("sequence_number"),
            recipient_id=data.get("recipient_id"),
        )

    def to_json(self) -> str:
        """Serialise to a JSON string."""
        return json.dumps(self.to_dict(), sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> "V2XMessage":
        """Parse a JSON string produced by :meth:`to_json`."""
        try:
            data = json.loads(text)
        except (TypeError, json.JSONDecodeError) as error:
            raise MessageValidationError(f"invalid JSON: {error}") from None
        return cls.from_dict(data)


# ------------------------------------------------------------------ factories
def _maneuver_between(lane: int, target_lane: Optional[int]) -> Maneuver:
    if target_lane is None or target_lane == lane:
        return Maneuver.KEEP_LANE
    return Maneuver.LANE_CHANGE_LEFT if target_lane < lane else Maneuver.LANE_CHANGE_RIGHT


def make_status_broadcast(vehicle: "Vehicle", timestamp: float, sequence_number: Optional[int] = None) -> V2XMessage:
    """Periodic vehicle status (position, speed, lane, current intention)."""
    return V2XMessage(
        sender_id=vehicle.vehicle_id,
        message_type=MessageType.VEHICLE_STATUS,
        timestamp=timestamp,
        position=vehicle.s,
        speed=vehicle.speed,
        lane=vehicle.lane,
        target_lane=vehicle.target_lane,
        intended_maneuver=_maneuver_between(vehicle.lane, vehicle.target_lane),
        sequence_number=sequence_number,
    )


def make_lane_change_request(
    vehicle: "Vehicle", target_lane: int, timestamp: float, sequence_number: Optional[int] = None
) -> V2XMessage:
    """Announce an intended lane change to neighbouring vehicles."""
    return V2XMessage(
        sender_id=vehicle.vehicle_id,
        message_type=MessageType.LANE_CHANGE_REQUEST,
        timestamp=timestamp,
        position=vehicle.s,
        speed=vehicle.speed,
        lane=vehicle.lane,
        target_lane=target_lane,
        intended_maneuver=_maneuver_between(vehicle.lane, target_lane),
        sequence_number=sequence_number,
    )


def make_lane_change_response(
    vehicle: "Vehicle",
    accepted: bool,
    target_lane: int,
    timestamp: float,
    recipient_id: str,
    sequence_number: Optional[int] = None,
) -> V2XMessage:
    """Acknowledge (accepted=True) or reject a lane-change request."""
    return V2XMessage(
        sender_id=vehicle.vehicle_id,
        message_type=MessageType.LANE_CHANGE_ACK if accepted else MessageType.LANE_CHANGE_REJECT,
        timestamp=timestamp,
        position=vehicle.s,
        speed=vehicle.speed,
        lane=vehicle.lane,
        target_lane=target_lane,
        intended_maneuver=Maneuver.NONE,
        sequence_number=sequence_number,
        recipient_id=recipient_id,
    )


def make_lane_change_complete(vehicle: "Vehicle", timestamp: float, sequence_number: Optional[int] = None) -> V2XMessage:
    """Notify others that the lane change finished (``lane`` is the new lane)."""
    return V2XMessage(
        sender_id=vehicle.vehicle_id,
        message_type=MessageType.LANE_CHANGE_COMPLETE,
        timestamp=timestamp,
        position=vehicle.s,
        speed=vehicle.speed,
        lane=vehicle.lane,
        target_lane=None,
        intended_maneuver=Maneuver.KEEP_LANE,
        sequence_number=sequence_number,
    )


def make_emergency_brake_warning(vehicle: "Vehicle", timestamp: float, sequence_number: Optional[int] = None) -> V2XMessage:
    """Warn followers that the sender started hard braking (extension message type)."""
    return V2XMessage(
        sender_id=vehicle.vehicle_id,
        message_type=MessageType.EMERGENCY_BRAKE_WARNING,
        timestamp=timestamp,
        position=vehicle.s,
        speed=vehicle.speed,
        lane=vehicle.lane,
        target_lane=vehicle.target_lane,
        intended_maneuver=Maneuver.KEEP_LANE,
        sequence_number=sequence_number,
    )

"""V2X message structures and the (future) communication interface.

NOTE: This is a Python-level model of V2X-style messages. It is NOT a
standards-compliant 5G NR-V2X (3GPP) / ETSI ITS-G5 / SAE J2735 implementation.
"""
from communication.messages import (
    Maneuver,
    MessageType,
    MessageValidationError,
    V2XMessage,
    make_emergency_brake_warning,
    make_lane_change_complete,
    make_lane_change_request,
    make_lane_change_response,
    make_status_broadcast,
)
from communication.v2x_interface import CommunicationEvent, NullV2XChannel, V2XInterface

__all__ = [
    "Maneuver", "MessageType", "MessageValidationError", "V2XMessage",
    "make_emergency_brake_warning", "make_lane_change_complete", "make_lane_change_request",
    "make_lane_change_response", "make_status_broadcast",
    "CommunicationEvent", "NullV2XChannel", "V2XInterface",
]

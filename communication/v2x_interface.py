"""Interface between vehicles and a (future) V2X network.

Phase 1 ships only :class:`NullV2XChannel`, which delivers NOTHING and merely
records that a message was handed to the interface. There is no network model,
no latency, no packet loss. A later phase will add a simulated channel that
implements :class:`V2XInterface`; it will still be a Python simulation, not a
standards-compliant 5G NR-V2X stack.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass

from communication.messages import V2XMessage


@dataclass(frozen=True)
class CommunicationEvent:
    """One entry in the communication log shown by the HUD."""

    time: float
    kind: str
    message: V2XMessage
    note: str = ""

    def describe(self) -> str:
        """One-line text for the HUD / logs."""
        seq = "" if self.message.sequence_number is None else f" #{self.message.sequence_number}"
        return f"t={self.time:6.1f}s {self.message.sender_id} {self.message.message_type.value}{seq} [{self.kind}]"


class V2XInterface(ABC):
    """Abstract V2X channel used by the simulation."""

    @property
    @abstractmethod
    def description(self) -> str:
        """Human-readable description of what this channel really does."""

    @abstractmethod
    def send(self, message: V2XMessage, now: float) -> None:
        """Hand a message to the channel at simulation time ``now``."""

    @abstractmethod
    def receive(self, vehicle_id: str, now: float) -> list[V2XMessage]:
        """Messages delivered to ``vehicle_id`` by time ``now``."""

    @abstractmethod
    def recent_events(self, limit: int = 5) -> list[CommunicationEvent]:
        """Most recent communication events, oldest first."""

    @abstractmethod
    def reset(self) -> None:
        """Clear queues and logs."""


class NullV2XChannel(V2XInterface):
    """Phase 1 placeholder channel: logs sends, delivers nothing."""

    def __init__(self, max_events: int = 50) -> None:
        self._events: deque[CommunicationEvent] = deque(maxlen=max_events)
        self.sent_count = 0

    @property
    def description(self) -> str:
        return "No network (Phase 1): logged, never delivered"

    def send(self, message: V2XMessage, now: float) -> None:
        self.sent_count += 1
        self._events.append(CommunicationEvent(now, "NOT DELIVERED", message, "no network in Phase 1"))

    def receive(self, vehicle_id: str, now: float) -> list[V2XMessage]:
        return []

    def recent_events(self, limit: int = 5) -> list[CommunicationEvent]:
        return list(self._events)[-limit:] if limit > 0 else []

    def reset(self) -> None:
        self._events.clear()
        self.sent_count = 0

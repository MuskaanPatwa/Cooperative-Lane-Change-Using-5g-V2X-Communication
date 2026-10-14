"""Road geometry: lanes, lateral positions and boundary validation."""
from __future__ import annotations

from dataclasses import dataclass

import config


@dataclass(frozen=True)
class Road:
    """A straight multi-lane highway.

    Longitudinal position ``s`` runs along the road; lateral position is the
    distance in metres from the left road edge. Lane 0 is the leftmost lane.
    """

    num_lanes: int = config.NUM_LANES
    lane_width: float = config.LANE_WIDTH_M

    def __post_init__(self) -> None:
        if self.num_lanes < 1:
            raise ValueError(f"Road needs at least one lane, got {self.num_lanes}")
        if self.lane_width <= 0:
            raise ValueError(f"Lane width must be positive, got {self.lane_width}")

    @property
    def width(self) -> float:
        """Total road width in metres."""
        return self.num_lanes * self.lane_width

    def is_valid_lane(self, lane: object) -> bool:
        """True if ``lane`` is an integer index of an existing lane."""
        return (
            isinstance(lane, int)
            and not isinstance(lane, bool)
            and 0 <= lane < self.num_lanes
        )

    def lane_center(self, lane: int) -> float:
        """Lateral position (m from left edge) of a lane centre."""
        if not self.is_valid_lane(lane):
            raise ValueError(
                f"Lane {lane!r} is outside the road (valid lanes: 0..{self.num_lanes - 1})"
            )
        return (lane + 0.5) * self.lane_width

    def lane_at(self, lateral: float) -> int:
        """Index of the lane containing a lateral position (clamped to the road)."""
        index = int(lateral // self.lane_width)
        return max(0, min(self.num_lanes - 1, index))

    def clamp_lateral(self, lateral: float, vehicle_width: float) -> float:
        """Clamp a lateral centre position so the vehicle body stays on the road."""
        half = vehicle_width / 2.0
        return max(half, min(self.width - half, lateral))

    def divider_positions(self) -> list[float]:
        """Lateral positions of the dashed lane dividers (excluding road edges)."""
        return [i * self.lane_width for i in range(1, self.num_lanes)]

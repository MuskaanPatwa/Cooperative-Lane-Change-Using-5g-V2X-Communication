"""The player-controlled EGO vehicle."""
from __future__ import annotations

from dataclasses import dataclass

import config
from vehicles.vehicle import Vehicle


@dataclass(eq=False)
class EgoVehicle(Vehicle):
    """EGO vehicle: a :class:`Vehicle` driven by throttle/brake commands."""

    color: config.Color = config.COLOR_EGO
    min_speed: float = config.EGO_MIN_SPEED_MPS
    max_speed: float = config.EGO_MAX_SPEED_MPS
    accel_command: float = config.EGO_ACCEL_CMD_MPS2
    brake_command: float = config.EGO_BRAKE_CMD_MPS2

    is_ego = True

    def command(self, accelerate: bool, brake: bool) -> None:
        """Apply pedal inputs. Braking wins if both are pressed; no input = coast."""
        if brake:
            self.set_acceleration(-self.brake_command)
        elif accelerate:
            self.set_acceleration(self.accel_command)
        else:
            self.set_acceleration(0.0)

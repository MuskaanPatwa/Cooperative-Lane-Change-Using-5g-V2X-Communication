"""Repeatable test scenarios for the batch runner.

Each scenario modifies a freshly reset :class:`Simulation` (``setup``) and may
act while it runs (``on_step``). The driver model used by the batch runner is
"hold the accelerator the whole time", so the assist must do the avoiding.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

from vehicles.traffic_vehicle import TrafficVehicle

if TYPE_CHECKING:
    from simulation.simulation import Simulation


@dataclass(frozen=True)
class Scenario:
    """A named test case."""

    name: str
    description: str
    setup: Callable[["Simulation"], None]
    on_step: Optional[Callable[["Simulation"], None]] = None


def _vehicle(sim: "Simulation", vehicle_id: str) -> TrafficVehicle:
    return next(v for v in sim.traffic if v.vehicle_id == vehicle_id)


def _place(sim: "Simulation", vehicle_id: str, lane: int, offset_m: float, speed: float) -> TrafficVehicle:
    vehicle = _vehicle(sim, vehicle_id)
    vehicle.place(lane, sim.ego.s + offset_m, speed)
    vehicle.desired_speed = speed
    return vehicle


def _add(sim: "Simulation", vehicle_id: str, lane: int, offset_m: float, speed: float) -> None:
    sim.traffic.append(TrafficVehicle(
        vehicle_id=vehicle_id, lane=lane, s=sim.ego.s + offset_m, speed=speed,
        desired_speed=speed, color=(170, 170, 200), road=sim.road,
    ))


def _baseline(sim: "Simulation") -> None:
    pass


def _slow_leader(sim: "Simulation") -> None:
    _place(sim, "T2", sim.ego.lane, 35.0, 8.0)


def _sudden_brake_setup(sim: "Simulation") -> None:
    _place(sim, "T2", sim.ego.lane, 25.0, 20.0)


def _sudden_brake_step(sim: "Simulation") -> None:
    leader = _vehicle(sim, "T2")
    if sim.time >= 3.0 and leader.desired_speed > 10.0:
        leader.desired_speed = 4.0           # leader brakes hard


def _boxed_in(sim: "Simulation") -> None:
    _place(sim, "T2", sim.ego.lane, 35.0, 10.0)
    _place(sim, "T1", 0, 3.0, 20.0)
    _place(sim, "T5", 2, -3.0, 20.0)


def _dense(sim: "Simulation") -> None:
    for vid, lane, offset, speed in (("D1", 0, 55.0, 19.0), ("D2", 2, 45.0, 21.0),
                                      ("D3", 1, -60.0, 24.0), ("D4", 0, -50.0, 26.0)):
        _add(sim, vid, lane, offset, speed)


SCENARIOS: dict[str, Scenario] = {s.name: s for s in (
    Scenario("baseline", "Default traffic, driver holds the accelerator", _baseline),
    Scenario("slow_leader", "Very slow car 35 m ahead in the EGO lane", _slow_leader),
    Scenario("sudden_brake", "Leader 25 m ahead brakes hard at t=3 s", _sudden_brake_setup, _sudden_brake_step),
    Scenario("boxed_in", "Slow leader ahead, cars alongside in both neighbouring lanes", _boxed_in),
    Scenario("dense_traffic", "Default traffic plus four extra cars", _dense),
)}

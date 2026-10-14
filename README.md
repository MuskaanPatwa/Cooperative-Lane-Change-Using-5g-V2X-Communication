# 5G V2X Cooperative Lane Change Simulation — Phase 1

Interactive 2D highway simulation (Python 3.11 + Pygame). **Phase 1 is the basic
road/vehicle framework**; cooperative V2X behaviour is *prepared for*, not implemented.

## Windows PowerShell setup

```powershell
# 1. Create the project directory and extract the ZIP contents into it
mkdir C:\Projects\v2x_cooperative_lane_change
cd C:\Projects\v2x_cooperative_lane_change
# (copy / extract the project files here so main.py is in this folder)

# 2. Create and activate a virtual environment (Python 3.11)
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
# If activation is blocked:  Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass

# 3. Install dependencies
python -m pip install --upgrade pip
pip install -r requirements.txt

# 4. Run the simulation
python main.py

# 5. Run the tests
pytest -q
```

## Controls
| Key | Action |
|---|---|
| Up / Down | accelerate / brake (hold) |
| Left / Right | request lane change (one request per key press) |
| Space | pause / resume |
| A | toggle automatic evasive lane change |
| G | show / hide live performance graphs |
| S | show / hide run summary card |
| E | export the recorded run to `logs/run_<time>.csv` |
| R | reset |
| Esc | exit |

Lane numbers on screen are 1-based (1 = leftmost); code uses 0-based indices.

## Architecture
```
config.py                 all tunable constants
simulation/road.py        lane geometry, boundary validation
simulation/simulation.py  owns vehicles, time stepping, traffic recycling, V2X logging (no pygame)
vehicles/vehicle.py       kinematics + smooth lane transition (smoothstep over 2.5 s)
vehicles/ego_vehicle.py   player vehicle (throttle/brake commands)
vehicles/traffic_vehicle.py  speed holding + basic gap-keeping
controllers/lane_change_controller.py  validates + starts lane changes
controllers/vehicle_controller.py      ControlInput -> EGO commands, last-action text
safety/safety_checker.py  gaps, closing speed, TTC, SAFE/UNSAFE/WAITING_FOR_GAP
safety/collision_avoidance.py  automatic slowing/braking for the EGO (CLEAR/SLOWING/BRAKING/EMERGENCY_BRAKING)
communication/messages.py       V2XMessage + validation + JSON + factories
communication/v2x_interface.py  V2XInterface (abstract) + NullV2XChannel
analysis/metrics.py       MetricsRecorder, RunSummary, CSV export (no pygame)
analysis/scenarios.py     5 repeatable test scenarios
run_batch.py              headless batch runner -> results table + logs/batch_results.csv
plot_results.py           CSV -> PNG charts (needs matplotlib)
visualization/graphs.py   live graph overlay, summary card, toast
visualization/renderer.py, hud.py   rendering (with graphs.py the only modules that import pygame)
main.py                   event loop, key mapping
```
Data flow per frame: `main.py` → `ControlInput` → `Simulation.step()` (fixed ≤20 ms
sub-steps, so movement is frame-rate independent) → `Renderer` + `Hud` read the state.

## Performance evaluation
- **In the window:** `G` live graphs (speed, gap to leader, TTC, avoidance-level strip), `S` summary card, `E` export CSV.
  On exit the run is saved automatically to `logs/` and a summary is printed.
- **Batch runs:** `python run_batch.py` (options `--duration 90 --seeds 3 --no-auto-lane`) runs the scenarios
  baseline, slow_leader, sudden_brake, boxed_in and dense_traffic with a driver who holds the accelerator the whole time,
  and prints min gap, min TTC, braking events, peak deceleration, evasive lane changes and collisions.
- **Charts for a report:** `pip install -r requirements-plots.txt`, then `python plot_results.py` (newest CSV in `logs/`).
- Dense-traffic runs are identical across seeds until traffic is respawned, so more seeds only matter in long runs.

## What is actually implemented
- 3-lane road, scrolling markings, EGO + 8 traffic vehicles with IDs/speeds/lane labels, camera locked to EGO.
- EGO speed limits 0–38 m/s (it can brake to a standstill); lane changes limited to adjacent, existing lanes; no new request during a transition; target lane tracked separately from current lane.
- Traffic: constant desired speed plus a simple braking rule behind a leader. Traffic does **not** change lanes. Vehicles that fall far behind/ahead are respawned (deterministic seed).
- **Collision avoidance assist (EGO):** automatically brakes (overriding the throttle) using the required deceleration
  `v_closing^2 / (2*(gap - 4 m))` with a 1.3 safety factor, plus gentle gap keeping (~1.2 s). When closing on a slower
  vehicle it also tries an evasive lane change into a neighbouring lane that passes the safety check (toggle with **A**).
  On hard braking the EGO creates an `EMERGENCY_BRAKE_WARNING` message (an extension type), which is logged but not
  delivered in Phase 1. Traffic reacts to the EGO by sensing the gap, not by messages. Tested: 0 EGO collisions in
  randomized multi-minute runs with sudden braking events (see `tests/test_avoidance.py`).
- Vehicles can never overlap: a simple bumper-contact rule holds the rear vehicle just behind the front one at its speed. EGO contacts are reported (COLLISION banner/counter). This is not crash physics.
- Safety checks (informative, shown in the HUD for the current lane and both neighbours):
  road boundary, target-slot occupancy, bumper-to-bumper front/rear gap, closing speed,
  constant-speed TTC. `SAFE` / `UNSAFE` / `WAITING_FOR_GAP`.
  By default (`ENFORCE_SAFETY_ON_LANE_CHANGE = True` in `config.py`) a lane change is rejected unless the check says SAFE; set it to `False` to make the checks informative only.
- V2X: message dataclass with validation and JSON round-trip; factories for status broadcast, lane-change request, ACK/REJECT and completion. The EGO's status (1 Hz), accepted requests and completions are created and logged in the HUD as **NOT DELIVERED**.
  `recipient_id` is a small optional extension beyond the specified fields.

## What is NOT implemented (honest limits)
- No V2X network: no latency, packet loss, delivery, or receivers. Cooperative negotiation is Phase 2.
- This is a Python model of V2X-style messages, **not** a standards-compliant 5G NR-V2X (3GPP) / ETSI / SAE implementation.
- The assist reads the leader's braking (like a radar's relative-acceleration estimate); it does not yet use V2X for this - that is a Phase 2 goal.
- The avoidance assist only looks forward, assumes the leader keeps its speed, and cannot stop a vehicle hitting the EGO
  from behind or avoid a leader that brakes harder than the EGO can. "Zero collisions" holds for the tested scenarios, not
  as a proof or a guarantee; it is not a validated automotive AEB/ACC system.
- The safety checker is a basic gap/TTC check, **not** a validated collision-prediction or real-world safety system.
- Traffic is not a validated car-following or autonomous-driving model.
- NumPy is not used (nothing needed it), keeping dependencies to Pygame + pytest.
- Verification so far: 123 pytest tests and a headless render/fuzz run on Linux; I have not been able to run the window on Windows 10/11 myself.

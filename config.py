"""Central configuration for the Phase 1 simulation.

Every tunable number lives here so that nothing is hard-coded in the logic
modules. Units: metres (m), seconds (s), metres per second (m/s) unless a
name says otherwise. Lane indices are 0-based internally (0 = leftmost lane);
the HUD displays them 1-based for humans.
"""
from __future__ import annotations

Color = tuple[int, int, int]

# --------------------------------------------------------------------------
# Window / rendering
# --------------------------------------------------------------------------
WINDOW_WIDTH = 1120
WINDOW_HEIGHT = 720
ROAD_VIEW_WIDTH = 640                      # left part: road, right part: HUD
HUD_WIDTH = WINDOW_WIDTH - ROAD_VIEW_WIDTH
FPS = 60
WINDOW_TITLE = "5G V2X Cooperative Lane Change - Phase 1 (basic framework)"
PIXELS_PER_METER = 14.0
EGO_SCREEN_Y_FRACTION = 0.72               # EGO is drawn at 72 % of window height
VISUAL_TILT_GAIN = 2.0                     # exaggerates heading tilt (drawing only)

LANE_DASH_LENGTH_M = 3.0
LANE_DASH_PERIOD_M = 12.0
ROADSIDE_POST_SPACING_M = 25.0

# Colours
COLOR_GRASS: Color = (34, 85, 45)
COLOR_ASPHALT: Color = (52, 54, 60)
COLOR_MARKING: Color = (235, 235, 235)
COLOR_EDGE_LINE: Color = (245, 205, 70)
COLOR_POST: Color = (200, 200, 200)
COLOR_EGO: Color = (0, 170, 255)
COLOR_HUD_BG: Color = (22, 24, 30)
COLOR_HUD_TEXT: Color = (230, 230, 235)
COLOR_HUD_DIM: Color = (140, 145, 155)
COLOR_HUD_ACCENT: Color = (90, 200, 255)
COLOR_SAFE: Color = (80, 220, 120)
COLOR_UNSAFE: Color = (255, 90, 90)
COLOR_WAITING: Color = (255, 190, 60)

# --------------------------------------------------------------------------
# Road
# --------------------------------------------------------------------------
NUM_LANES = 3
LANE_WIDTH_M = 3.7

# --------------------------------------------------------------------------
# Vehicles (shared)
# --------------------------------------------------------------------------
VEHICLE_LENGTH_M = 4.5
VEHICLE_WIDTH_M = 1.9
MAX_ACCEL_MPS2 = 3.0                       # hard physical limits for any vehicle
MAX_DECEL_MPS2 = 8.0
LANE_CHANGE_DURATION_S = 2.5               # time for a full lane transition

# --------------------------------------------------------------------------
# EGO vehicle
# --------------------------------------------------------------------------
EGO_ID = "EGO"
EGO_INITIAL_LANE = 1
EGO_INITIAL_S = 0.0
EGO_INITIAL_SPEED_MPS = 20.0
EGO_MIN_SPEED_MPS = 0.0
EGO_MAX_SPEED_MPS = 38.0
EGO_ACCEL_CMD_MPS2 = 3.0                   # Up Arrow
EGO_BRAKE_CMD_MPS2 = 6.0                   # Down Arrow

# --------------------------------------------------------------------------
# Surrounding traffic
# --------------------------------------------------------------------------
TRAFFIC_MIN_SPEED_MPS = 0.0
TRAFFIC_MAX_SPEED_MPS = 40.0
TRAFFIC_SPEED_GAIN = 0.5                   # 1/s, pull towards desired speed
TRAFFIC_TIME_HEADWAY_S = 1.5
TRAFFIC_STANDSTILL_GAP_M = 5.0
TRAFFIC_GAP_GAIN = 0.25                    # 1/s^2
TRAFFIC_REL_SPEED_GAIN = 0.8               # 1/s

# (vehicle_id, lane, longitudinal offset from EGO start [m], speed [m/s], colour)
INITIAL_TRAFFIC: tuple[tuple[str, int, float, float, Color], ...] = (
    ("T1", 0, 30.0, 24.0, (230, 80, 80)),
    ("T2", 1, 45.0, 17.0, (240, 200, 60)),
    ("T3", 2, 20.0, 27.0, (150, 110, 230)),
    ("T4", 0, -25.0, 28.0, (80, 200, 140)),
    ("T5", 2, -15.0, 22.0, (240, 140, 60)),
    ("T6", 1, -40.0, 26.0, (220, 100, 190)),
    ("T7", 2, 70.0, 20.0, (100, 160, 240)),
    ("T8", 0, 90.0, 22.0, (190, 190, 190)),
)
TRAFFIC_SPEED_RANGE_MPS = (16.0, 30.0)     # speeds used when traffic is recycled

# Traffic recycling keeps a roughly constant density around the EGO.
TRAFFIC_RECYCLE_BEHIND_M = 120.0
TRAFFIC_RECYCLE_AHEAD_M = 170.0
SPAWN_AHEAD_RANGE_M = (90.0, 140.0)
SPAWN_BEHIND_RANGE_M = (-110.0, -80.0)
SPAWN_MIN_GAP_M = 30.0
SPAWN_ATTEMPTS = 20
TRAFFIC_COLORS: tuple[Color, ...] = tuple(t[4] for t in INITIAL_TRAFFIC)

# --------------------------------------------------------------------------
# Safety checks (informative in Phase 1)
# --------------------------------------------------------------------------
SAFETY_MIN_FRONT_GAP_M = 10.0
SAFETY_MIN_REAR_GAP_M = 10.0
SAFETY_MIN_TTC_S = 3.0
SAFETY_CLOSING_SPEED_EPS_MPS = 0.1         # below this, vehicles are "not closing"
ENFORCE_SAFETY_ON_LANE_CHANGE = True       # block lane changes that are not SAFE
CONTACT_MARGIN_M = 0.05                    # gap left when a rear vehicle is held behind a front one

# --------------------------------------------------------------------------
# Collision avoidance assist (EGO only)
# --------------------------------------------------------------------------
AVOID_ENABLED = True                       # automatic braking / gap keeping
AVOID_AUTO_LANE_CHANGE = True              # evasive lane change (toggle with A)
AVOID_STOP_MARGIN_M = 4.0                  # braking aims to stop closing before this gap
AVOID_BRAKE_FACTOR = 1.3                   # safety factor on the required deceleration
AVOID_FOLLOW_TIME_S = 1.2                  # desired time gap to the vehicle ahead
AVOID_FOLLOW_MIN_GAP_M = 6.0
AVOID_MIN_ACTIVE_DECEL_MPS2 = 0.5          # below this the assist stays silent
AVOID_BRAKING_DECEL_MPS2 = 2.0             # level BRAKING from here
AVOID_EMERGENCY_DECEL_MPS2 = 5.0           # level EMERGENCY_BRAKING from here
AVOID_LEAD_BRAKE_FACTOR = 1.1              # feed-forward on the leader's own braking
AVOID_LEAD_BRAKE_RANGE_FACTOR = 2.0        # only react to leader braking within 2x desired gap
AVOID_CLOSING_EPS_MPS = 0.5                # "closing" = approaching faster than this
AVOID_EVASION_COOLDOWN_S = 4.0             # after an accepted evasive lane change
AVOID_EVASION_RETRY_S = 0.5                # after no safe lane was available

# --------------------------------------------------------------------------
# Simulation loop
# --------------------------------------------------------------------------
RANDOM_SEED = 42
PHYSICS_STEP_S = 0.02                      # maximum integration sub-step
MAX_FRAME_DT_S = 0.1                       # longer frames are clipped
STATUS_BROADCAST_PERIOD_S = 1.0            # EGO status messages (logged only)

# --------------------------------------------------------------------------
# Performance analysis (metrics, graphs, export)
# --------------------------------------------------------------------------
METRICS_SAMPLE_PERIOD_S = 0.1              # one recorded sample every 0.1 s of sim time
GRAPH_WINDOW_S = 30.0                      # time span shown by the live graphs
GRAPH_GAP_MAX_M = 100.0                    # y-axis cap for the gap plot
GRAPH_TTC_MAX_S = 10.0                     # y-axis cap for the TTC plot
LOG_DIR = "logs"                           # CSV exports go here

"""Information panel (right side of the window)."""
from __future__ import annotations

import math

import pygame

import config
from safety.safety_checker import SafetyResult, SafetyStatus
from simulation.simulation import Simulation
from visualization.renderer import make_font


def format_distance(value_m: float) -> str:
    """'--' for 'no vehicle', otherwise metres with one decimal."""
    return "--" if not math.isfinite(value_m) else f"{value_m:.1f} m"


def format_ttc(value_s: float) -> str:
    """'--' for 'not closing', otherwise seconds with one decimal."""
    return "--" if not math.isfinite(value_s) else f"{value_s:.1f} s"


STATUS_COLORS = {
    SafetyStatus.SAFE: config.COLOR_SAFE,
    SafetyStatus.UNSAFE: config.COLOR_UNSAFE,
    SafetyStatus.WAITING_FOR_GAP: config.COLOR_WAITING,
}


class Hud:
    """Draws EGO state, basic safety checks, controls and the reserved V2X panel."""

    LINE_HEIGHT = 18
    MARGIN = 16

    def __init__(self, screen: pygame.Surface) -> None:
        self.screen = screen
        self.rect = pygame.Rect(config.ROAD_VIEW_WIDTH, 0, config.HUD_WIDTH, config.WINDOW_HEIGHT)
        self.title_font = make_font(18, bold=True)
        self.head_font = make_font(14, bold=True)
        self.font = make_font(14)
        self._y = 0

    # ---------------------------------------------------------------- helpers
    def _line(self, text: str, color: config.Color = config.COLOR_HUD_TEXT, font: pygame.font.Font | None = None, indent: int = 0) -> None:
        font = font or self.font
        max_width = self.rect.width - 2 * self.MARGIN - indent
        while text and font.size(text)[0] > max_width:
            text = text[:-2].rstrip() + "…"
        self.screen.blit(font.render(text, True, color), (self.rect.x + self.MARGIN + indent, self._y))
        self._y += self.LINE_HEIGHT

    def _heading(self, text: str) -> None:
        self._y += 6
        self._line(text, config.COLOR_HUD_ACCENT, self.head_font)
        pygame.draw.line(self.screen, (60, 66, 80), (self.rect.x + self.MARGIN, self._y - 2), (self.rect.right - self.MARGIN, self._y - 2))
        self._y += 2

    def _safety_line(self, label: str, result: SafetyResult) -> None:
        color = STATUS_COLORS[result.status]
        if result.out_of_road:
            self._line(f"{label}: N/A (road edge)", config.COLOR_HUD_DIM)
            return
        self._line(f"{label}: {result.status.value}", color)
        gaps = result.gaps
        if gaps is not None:
            self._line(f"front {format_distance(gaps.front_gap)} TTC {format_ttc(gaps.front_ttc)}", config.COLOR_HUD_DIM, indent=14)
            self._line(f"rear  {format_distance(gaps.rear_gap)} TTC {format_ttc(gaps.rear_ttc)}", config.COLOR_HUD_DIM, indent=14)
        if result.reasons:
            self._line(result.reasons[0], color, indent=14)

    # ------------------------------------------------------------------- draw
    def draw(self, sim: Simulation) -> None:
        """Draw the full information panel."""
        pygame.draw.rect(self.screen, config.COLOR_HUD_BG, self.rect)
        pygame.draw.line(self.screen, (70, 76, 92), self.rect.topleft, self.rect.bottomleft, 2)
        self._y = 12
        ego = sim.ego

        self._line("V2X Cooperative Lane Change", config.COLOR_HUD_TEXT, self.title_font)
        self._y += 2
        self._line("Phase 1: basic framework (no V2X network)", config.COLOR_HUD_DIM)

        self._heading("EGO VEHICLE")
        self._line(f"ID:         {ego.vehicle_id}")
        self._line(f"Speed:      {ego.speed:5.1f} m/s | {ego.speed * 3.6:5.1f} km/h")
        self._line(f"Lane:       {ego.lane + 1}   (1 = leftmost)")
        target = "no change" if ego.target_lane is None else str(ego.target_lane + 1)
        self._line(f"Target:     {target}")
        progress = f" {ego.lane_change_progress * 100:3.0f}%" if ego.lane_change_in_progress else ""
        self._line(f"Maneuver:   {ego.maneuver.value}{progress}")
        self._line(f"Last action: {sim.vehicle_controller.last_action}")
        self._line(f"Surrounding vehicles: {len(sim.traffic)}")
        state = "  [PAUSED]" if sim.paused else ""
        self._line(f"Sim time:   {sim.time:6.1f} s{state}", config.COLOR_WAITING if sim.paused else config.COLOR_HUD_TEXT)
        if sim.collision_with:
            self._line(f"COLLISION with {sim.collision_with}! (count {sim.collision_count})", config.COLOR_UNSAFE, self.head_font)

        mode = "unsafe changes blocked" if sim.lane_change_controller.enforce_safety else "informative only"
        self._heading(f"BASIC SAFETY CHECKS ({mode})")
        own = sim.safety_report.own_lane
        ahead = own.front_vehicle_id or "none"
        behind = own.rear_vehicle_id or "none"
        self._line(f"Ahead  ({ahead}): {format_distance(own.front_gap)}  TTC {format_ttc(own.front_ttc)}")
        self._line(f"Behind ({behind}): {format_distance(own.rear_gap)}  TTC {format_ttc(own.rear_ttc)}")
        self._safety_line("Left  lane", sim.safety_report.left)
        self._safety_line("Right lane", sim.safety_report.right)

        self._heading("COLLISION AVOIDANCE")
        avoid = sim.vehicle_controller.last_avoidance
        colors = {"CLEAR": config.COLOR_SAFE, "SLOWING": config.COLOR_WAITING,
                  "BRAKING": config.COLOR_UNSAFE, "EMERGENCY_BRAKING": config.COLOR_UNSAFE}
        brake = f"  brake {avoid.decel:.1f} m/s2" if avoid.decel > 0 else ""
        self._line(f"Status: {avoid.level.value}{brake}", colors[avoid.level.value], self.head_font)
        leader = avoid.front_vehicle_id or "none"
        self._line(f"Leader {leader}: {format_distance(avoid.front_gap)}  TTC {format_ttc(avoid.ttc)}", config.COLOR_HUD_DIM)
        auto = "ON" if sim.vehicle_controller.auto_lane_change else "OFF"
        self._line(f"Auto evasive lane change: {auto} (key A)", config.COLOR_HUD_DIM)

        self._heading("CONTROLS")
        for text in ("Up/Down: accelerate / brake", "Left/Right: request lane change", "Space: pause  R: reset  A: auto-lane  Esc",
                     "G: graphs  S: run summary  E: export CSV"):
            self._line(text, config.COLOR_HUD_DIM)

        self._heading("V2X MESSAGES (reserved panel)")
        self._line(sim.v2x.description, config.COLOR_HUD_DIM)
        events = sim.v2x.recent_events(4)
        if not events:
            self._line("(no events yet)", config.COLOR_HUD_DIM)
        for event in events:
            self._line(event.describe(), config.COLOR_HUD_TEXT)

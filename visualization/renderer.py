"""Road and vehicle rendering with a camera that follows the EGO vehicle."""
from __future__ import annotations

import math

import pygame

import config
from simulation.road import Road
from simulation.simulation import Simulation
from vehicles.vehicle import Vehicle


def make_font(size: int, bold: bool = False) -> pygame.font.Font:
    """Monospace system font with a safe fallback."""
    return pygame.font.SysFont("consolas,dejavusansmono,couriernew,monospace", size, bold=bold)


class Renderer:
    """Draws the scrolling road view (left part of the window)."""

    def __init__(self, screen: pygame.Surface, road: Road) -> None:
        self.screen = screen
        self.road = road
        self.ppm = config.PIXELS_PER_METER
        self.height = config.WINDOW_HEIGHT
        self.view_rect = pygame.Rect(0, 0, config.ROAD_VIEW_WIDTH, self.height)
        self.ego_y = int(self.height * config.EGO_SCREEN_Y_FRACTION)
        self.road_px = road.width * self.ppm
        self.road_left = (config.ROAD_VIEW_WIDTH - self.road_px) / 2.0
        self.label_font = make_font(12, bold=True)

    # ---------------------------------------------------------------- camera
    def world_to_screen(self, s: float, lateral: float, ego_s: float) -> tuple[float, float]:
        """Map (longitudinal, lateral) metres to pixels; the EGO stays at a fixed screen row."""
        return (self.road_left + lateral * self.ppm, self.ego_y - (s - ego_s) * self.ppm)

    # ------------------------------------------------------------------ draw
    def draw(self, sim: Simulation) -> None:
        """Draw the whole road view."""
        ego_s = sim.ego.s
        self.screen.set_clip(self.view_rect)
        self.screen.fill(config.COLOR_GRASS, self.view_rect)
        self._draw_roadside_posts(ego_s)
        pygame.draw.rect(self.screen, config.COLOR_ASPHALT, (self.road_left, 0, self.road_px, self.height))
        self._draw_target_lane(sim)
        self._draw_markings(ego_s)
        for vehicle in sim.traffic:
            self._draw_vehicle(vehicle, ego_s, is_ego=False)
        self._draw_vehicle(sim.ego, ego_s, is_ego=True)
        self.screen.set_clip(None)

    def _s_range(self, ego_s: float) -> tuple[float, float]:
        top = ego_s + self.ego_y / self.ppm
        bottom = ego_s - (self.height - self.ego_y) / self.ppm
        return bottom, top

    def _draw_roadside_posts(self, ego_s: float) -> None:
        bottom, top = self._s_range(ego_s)
        spacing = config.ROADSIDE_POST_SPACING_M
        s = math.floor(bottom / spacing) * spacing
        left_x = self.road_left - 22
        right_x = self.road_left + self.road_px + 22
        while s < top + spacing:
            y = self.ego_y - (s - ego_s) * self.ppm
            for x in (left_x, right_x):
                pygame.draw.rect(self.screen, config.COLOR_POST, (x - 3, y - 9, 6, 18), border_radius=2)
            s += spacing

    def _draw_markings(self, ego_s: float) -> None:
        bottom, top = self._s_range(ego_s)
        period, dash = config.LANE_DASH_PERIOD_M, config.LANE_DASH_LENGTH_M
        s = math.floor(bottom / period) * period
        while s < top + period:
            y_top = self.ego_y - (s + dash - ego_s) * self.ppm
            y_bottom = self.ego_y - (s - ego_s) * self.ppm
            for lateral in self.road.divider_positions():
                x = self.road_left + lateral * self.ppm
                pygame.draw.rect(self.screen, config.COLOR_MARKING, (x - 1.5, y_top, 3, y_bottom - y_top))
            s += period
        for x in (self.road_left, self.road_left + self.road_px):
            pygame.draw.rect(self.screen, config.COLOR_EDGE_LINE, (x - 2, 0, 4, self.height))

    def _draw_target_lane(self, sim: Simulation) -> None:
        target = sim.ego.target_lane
        if target is None:
            return
        lane_px = self.road.lane_width * self.ppm
        tint = pygame.Surface((int(lane_px), self.height), pygame.SRCALPHA)
        tint.fill((*config.COLOR_EGO, 45))
        self.screen.blit(tint, (self.road_left + target * lane_px, 0))

    def _draw_vehicle(self, vehicle: Vehicle, ego_s: float, is_ego: bool) -> None:
        cx, cy = self.world_to_screen(vehicle.s, vehicle.lateral, ego_s)
        length_px, width_px = vehicle.length * self.ppm, vehicle.width * self.ppm
        if cy < -length_px - 40 or cy > self.height + length_px + 40:
            return

        pad = 4
        surf = pygame.Surface((int(width_px) + 2 * pad, int(length_px) + 2 * pad), pygame.SRCALPHA)
        body = pygame.Rect(pad, pad, int(width_px), int(length_px))
        pygame.draw.rect(surf, vehicle.color, body, border_radius=6)
        glass = (25, 30, 40)
        pygame.draw.rect(surf, glass, (body.x + 3, body.y + int(length_px * 0.18), body.w - 6, int(length_px * 0.2)), border_radius=3)
        pygame.draw.rect(surf, glass, (body.x + 3, body.y + int(length_px * 0.72), body.w - 6, int(length_px * 0.12)), border_radius=3)
        for x in (body.x + 3, body.right - 7):
            pygame.draw.rect(surf, (255, 245, 190), (x, body.y + 1, 4, 3))
        pygame.draw.rect(surf, (255, 255, 255) if is_ego else (15, 15, 15), body, width=3 if is_ego else 1, border_radius=6)

        tilt = -math.degrees(math.atan2(vehicle.lateral_speed, max(vehicle.speed, 1.0))) * config.VISUAL_TILT_GAIN
        if abs(tilt) > 0.05:
            surf = pygame.transform.rotate(surf, tilt)
        self.screen.blit(surf, surf.get_rect(center=(cx, cy)))
        self._draw_label(vehicle, cx, cy - length_px / 2.0, is_ego)

    def _draw_label(self, vehicle: Vehicle, cx: float, top_y: float, is_ego: bool) -> None:
        name = "EGO" if is_ego else vehicle.vehicle_id
        color = config.COLOR_EGO if is_ego else config.COLOR_HUD_TEXT
        lines = (f"{name} {vehicle.speed * 3.6:3.0f} km/h", f"lane {vehicle.lane + 1}")
        y = top_y - 4 - 14 * len(lines)
        for line in lines:
            text = self.label_font.render(line, True, color)
            backing = pygame.Surface((text.get_width() + 6, text.get_height() + 1), pygame.SRCALPHA)
            backing.fill((0, 0, 0, 150))
            rect = backing.get_rect(midtop=(cx, y))
            self.screen.blit(backing, rect)
            self.screen.blit(text, text.get_rect(center=rect.center))
            y += 14

"""Live performance graphs, run-summary card and toast messages (pygame)."""
from __future__ import annotations

from typing import Callable, Optional

import pygame

import config
from analysis.metrics import MetricsRecorder, Sample
from safety.collision_avoidance import AvoidanceLevel
from visualization.renderer import make_font

LEVEL_COLORS = {
    AvoidanceLevel.CLEAR: (60, 170, 100),
    AvoidanceLevel.SLOWING: (240, 190, 60),
    AvoidanceLevel.BRAKING: (240, 110, 60),
    AvoidanceLevel.EMERGENCY_BRAKING: (240, 60, 60),
}
PLOT_BG = (28, 32, 42, 235)
GRID = (70, 76, 92)


class GraphOverlay:
    """Scrolling plots of speed, gap to the leader and TTC over the last N seconds."""

    PLOT_HEIGHT = 86
    MARGIN = 10

    def __init__(self, surface: pygame.Surface, width: int = config.ROAD_VIEW_WIDTH) -> None:
        self.surface = surface
        self.width = width
        self.height = 30 + 3 * (self.PLOT_HEIGHT + 8) + 32
        self.font = make_font(12)
        self.title_font = make_font(14, bold=True)

    def draw(self, recorder: MetricsRecorder) -> None:
        """Draw the overlay in the top-left corner of the surface."""
        panel = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        panel.fill((10, 12, 18, 215))
        samples = recorder.samples
        window = config.GRAPH_WINDOW_S
        t1 = samples[-1].time if samples else 0.0
        t0 = t1 - window
        recent = self._recent(samples, t0)

        panel.blit(self.title_font.render(f"LIVE PERFORMANCE (last {window:.0f} s)  -  G to hide", True, config.COLOR_HUD_ACCENT), (self.MARGIN, 6))
        x = self.MARGIN
        w = self.width - 2 * self.MARGIN
        y = 28
        specs = (
            ("Speed", "km/h", lambda s: s.speed * 3.6, config.EGO_MAX_SPEED_MPS * 3.6, None, (90, 200, 255)),
            ("Gap to leader", "m", lambda s: s.front_gap, config.GRAPH_GAP_MAX_M, config.SAFETY_MIN_FRONT_GAP_M, (120, 230, 140)),
            ("TTC", "s", lambda s: s.front_ttc, config.GRAPH_TTC_MAX_S, config.SAFETY_MIN_TTC_S, (255, 190, 90)),
        )
        for title, unit, getter, y_max, threshold, color in specs:
            self._plot(panel, pygame.Rect(x, y, w, self.PLOT_HEIGHT), title, unit, recent, getter, t0, t1, y_max, threshold, color)
            y += self.PLOT_HEIGHT + 8
        self._level_strip(panel, pygame.Rect(x, y - 2, w, 8), recent, t0, t1)
        panel.blit(self.font.render("avoidance:", True, config.COLOR_HUD_DIM), (x, y + 8))
        lx = x + 80
        for level, color in LEVEL_COLORS.items():
            pygame.draw.rect(panel, color, (lx, y + 10, 10, 10))
            label = self.font.render(level.value.replace("_BRAKING", ""), True, config.COLOR_HUD_DIM)
            panel.blit(label, (lx + 14, y + 8))
            lx += 24 + label.get_width()
        self.surface.blit(panel, (0, 0))

    @staticmethod
    def _recent(samples: list[Sample], t0: float) -> list[Sample]:
        out: list[Sample] = []
        for sample in reversed(samples):
            if sample.time < t0:
                break
            out.append(sample)
        out.reverse()
        return out

    def _plot(self, panel, rect, title, unit, samples, getter: Callable[[Sample], Optional[float]],
              t0, t1, y_max, threshold, color) -> None:
        pygame.draw.rect(panel, PLOT_BG, rect)
        pygame.draw.rect(panel, GRID, rect, 1)

        def to_px(t: float, value: float) -> tuple[float, float]:
            fx = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            fy = max(0.0, min(1.0, value / y_max))
            return rect.x + fx * rect.w, rect.bottom - 2 - fy * (rect.h - 4)

        if threshold is not None:
            ty = to_px(t0, threshold)[1]
            pygame.draw.line(panel, (200, 90, 90), (rect.x, ty), (rect.right, ty), 1)
        run: list[tuple[float, float]] = []
        for sample in samples:
            value = getter(sample)
            if value is None:                       # no leader / not closing: break the line
                if len(run) > 1:
                    pygame.draw.lines(panel, color, False, run, 2)
                run = []
            else:
                run.append(to_px(sample.time, value))
        if len(run) > 1:
            pygame.draw.lines(panel, color, False, run, 2)

        latest = getter(samples[-1]) if samples else None
        shown = "--" if latest is None else f"{latest:.1f}"
        panel.blit(self.font.render(f"{title} [{unit}]  now: {shown}   (max {y_max:.0f})", True, config.COLOR_HUD_TEXT), (rect.x + 6, rect.y + 3))

    @staticmethod
    def _level_strip(panel, rect, samples, t0, t1) -> None:
        if len(samples) < 2 or t1 <= t0:
            return
        for current, following in zip(samples, samples[1:]):
            x_a = rect.x + (current.time - t0) / (t1 - t0) * rect.w
            x_b = rect.x + (following.time - t0) / (t1 - t0) * rect.w
            pygame.draw.rect(panel, LEVEL_COLORS[current.level], (x_a, rect.y, max(1.0, x_b - x_a + 1), rect.h))


class SummaryCard:
    """Full-height card with the run summary, drawn over the road view."""

    def __init__(self, surface: pygame.Surface, width: int = config.ROAD_VIEW_WIDTH, height: int = config.WINDOW_HEIGHT) -> None:
        self.surface = surface
        self.width, self.height = width, height
        self.font = make_font(16)
        self.title_font = make_font(20, bold=True)

    def draw(self, recorder: MetricsRecorder) -> None:
        panel = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        panel.fill((10, 12, 18, 235))
        panel.blit(self.title_font.render("RUN SUMMARY", True, config.COLOR_HUD_ACCENT), (24, 20))
        panel.blit(self.font.render("S: close   E: export CSV", True, config.COLOR_HUD_DIM), (24, 48))
        summary = recorder.summary()
        y = 90
        for label, value in summary.format_lines():
            bad = label == "Collisions" and summary.collisions > 0
            color = config.COLOR_UNSAFE if bad else config.COLOR_HUD_TEXT
            panel.blit(self.font.render(label, True, config.COLOR_HUD_DIM), (24, y))
            panel.blit(self.font.render(value, True, color), (300, y))
            y += 28
        self.surface.blit(panel, (0, 0))


def draw_toast(surface: pygame.Surface, text: str, width: int = config.ROAD_VIEW_WIDTH, height: int = config.WINDOW_HEIGHT) -> None:
    """Small message strip at the bottom of the road view."""
    font = make_font(14, bold=True)
    label = font.render(text, True, (255, 255, 255))
    box = pygame.Surface((min(width - 20, label.get_width() + 20), label.get_height() + 12), pygame.SRCALPHA)
    box.fill((20, 90, 160, 230))
    box.blit(label, (10, 6))
    surface.blit(box, ((width - box.get_width()) // 2, height - box.get_height() - 12))

"""Entry point: ``python main.py`` opens the interactive simulation window.

Optional: ``python main.py --max-frames 300`` quits automatically after N frames
(useful for smoke tests; combine with SDL_VIDEODRIVER=dummy for headless runs).
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional

import pygame

import config
from analysis.metrics import MetricsRecorder
from controllers.lane_change_controller import LaneDirection
from controllers.vehicle_controller import ControlInput
from simulation.simulation import Simulation
from visualization.graphs import GraphOverlay, SummaryCard, draw_toast
from visualization.hud import Hud
from visualization.renderer import Renderer

TOAST_SECONDS = 3.0


def run(max_frames: Optional[int] = None) -> int:
    """Run the main loop. Returns a process exit code."""
    pygame.init()
    try:
        screen = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
    except pygame.error as error:
        print(f"Could not open a window: {error}", file=sys.stderr)
        pygame.quit()
        return 1
    pygame.display.set_caption(config.WINDOW_TITLE)
    clock = pygame.time.Clock()

    sim = Simulation()
    renderer = Renderer(screen, sim.road)
    hud = Hud(screen)
    recorder = MetricsRecorder()
    graphs = GraphOverlay(screen)
    summary_card = SummaryCard(screen)
    show_graphs = False
    show_summary = False
    toast_text = ""
    toast_until = 0

    running = True
    frames = 0
    while running:
        dt = clock.tick(config.FPS) / 1000.0
        lane_request: Optional[LaneDirection] = None

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sim.toggle_pause()
                elif event.key == pygame.K_r:
                    sim.reset()
                    recorder.reset()
                elif event.key == pygame.K_a:
                    sim.vehicle_controller.auto_lane_change = not sim.vehicle_controller.auto_lane_change
                elif event.key == pygame.K_g:
                    show_graphs = not show_graphs
                elif event.key == pygame.K_s:
                    show_summary = not show_summary
                elif event.key == pygame.K_e:
                    if recorder.samples:
                        toast_text = f"Saved {recorder.write_csv()}"
                    else:
                        toast_text = "Nothing recorded yet"
                    toast_until = pygame.time.get_ticks() + int(TOAST_SECONDS * 1000)
                elif event.key == pygame.K_LEFT:
                    lane_request = LaneDirection.LEFT
                elif event.key == pygame.K_RIGHT:
                    lane_request = LaneDirection.RIGHT

        keys = pygame.key.get_pressed()
        control = ControlInput(accelerate=bool(keys[pygame.K_UP]), brake=bool(keys[pygame.K_DOWN]), lane_request=lane_request)
        sim.step(dt, control)
        recorder.update(sim)

        renderer.draw(sim)
        if show_graphs:
            graphs.draw(recorder)
        if show_summary:
            summary_card.draw(recorder)
        if toast_text and pygame.time.get_ticks() < toast_until:
            draw_toast(screen, toast_text)
        hud.draw(sim)
        pygame.display.flip()

        frames += 1
        if max_frames is not None and frames >= max_frames:
            running = False

    pygame.quit()
    if recorder.samples:
        path = recorder.write_csv()
        print(f"\nRun summary (saved {path}):")
        for label, value in recorder.summary().format_lines():
            print(f"  {label:<26}{value}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="5G V2X cooperative lane change simulation (Phase 1)")
    parser.add_argument("--max-frames", type=int, default=None, help="exit automatically after this many frames")
    args = parser.parse_args()
    sys.exit(run(args.max_frames))


if __name__ == "__main__":
    main()

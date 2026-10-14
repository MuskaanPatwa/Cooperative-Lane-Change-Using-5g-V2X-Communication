# Add-on guide: performance graphs, summary, CSV export, batch runner

Do this on a COPY or after a backup so nothing can be lost.

## Step 0 - Back up
Copy your whole `v2x_cooperative_lane_change` folder to `v2x_cooperative_lane_change_backup`.

## Step 1 - Add the NEW files (nothing is overwritten)
| File | Put it here |
|---|---|
| `analysis/__init__.py` | new folder `analysis/` in the project root |
| `analysis/metrics.py` | `analysis/` |
| `analysis/scenarios.py` | `analysis/` |
| `visualization/graphs.py` | existing `visualization/` folder |
| `run_batch.py` | project root (next to `main.py`) |
| `plot_results.py` | project root |
| `requirements-plots.txt` | project root |
| `tests/test_metrics.py` | existing `tests/` folder |

## Step 2 - REPLACE these existing files (they contain small edits)
| File | What changed |
|---|---|
| `config.py` | new constants appended (analysis); `EGO_MIN_SPEED_MPS` is now 0.0; new avoidance constants for leader-braking feed-forward |
| `safety/collision_avoidance.py` | leader-braking feed-forward added |
| `controllers/vehicle_controller.py` | `ControllerStats` counters (lane-change statistics) |
| `controllers/__init__.py` | exports `ControllerStats` |
| `visualization/hud.py` | one extra controls line (G / S / E) |
| `main.py` | recorder, G/S/E keys, auto-save + summary on exit |
| `tests/test_avoidance.py` | 2 extra tests |
| `README.md` | documentation only (optional) |

Tip: replace `config.py` LAST-but-before-testing only if you edited it yourself; if you have personal changes in it,
copy just the new blocks at the bottom and the `EGO_MIN_SPEED_MPS` / `AVOID_LEAD_BRAKE_*` lines.

## Step 3 - Install the optional plotting library
```powershell
.\.venv\Scripts\Activate.ps1
pip install -r requirements-plots.txt
```

## Step 4 - Check nothing broke
```powershell
pytest -q            # expect: 123 passed
python main.py       # press G (graphs), S (summary), E (export CSV), A (auto lane change)
```

## Step 5 - Try the new tools
```powershell
python run_batch.py                      # results table for all scenarios
python plot_results.py                   # PNG chart of the newest logs\run_*.csv
```

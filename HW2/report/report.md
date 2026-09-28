# HW2 Tuning Log

Raw tuning results behind [`report_hw2.tex`](report_hw2.tex).
All runs use the bicycle kinematic model with the PID longitudinal controller.
Pure pursuit always runs with `Lfc=1.0`, which `navigation.py` passes to the controller (overriding the constructor default of 5.0).

## Code changes

- `navigation.py`: the OpenCV display window was replaced with `imageio.get_writer()`, so the simulation saves an MP4 (`nav_output_{simulator}_{controller}_{track}_test.mp4`) and runs on a headless server.
  The main loop exits when the metrics evaluator sets `has_finished`, instead of waiting for Esc.

## Silverstone

| Controller | Gain | Lap time (s) | Avg cross-track error (m) |
|---|---|---:|---:|
| Pure pursuit | `kp=1.0` | 53.40 | 2.3545 |
| Pure pursuit | `kp=0.3` | 55.25 | 0.0836 |
| Pure pursuit | **`kp=0.1`** | 55.50 | **0.0265** |
| Pure pursuit | `kp=0.08` | 55.10 | 0.0278 |
| Pure pursuit | `kp=0.06` | 55.30 | 0.1754 |
| Stanley | `kp=5.0` | 56.00 | 0.2815 |
| Stanley | `kp=1.0` | 55.10 | 0.2739 |
| Stanley | `kp=0.1` | 55.20 | 0.8002 |
| LQR | default | 55.50 | 0.2064 |

## Monza

| Controller | Gain | Lap time (s) | Avg cross-track error (m) |
|---|---|---:|---:|
| Pure pursuit | `kp=0.1` | 47.45 | 0.0250 |
| Stanley | `kp=2.0` | 47.65 | 0.2647 |
| LQR | default | 47.50 | 0.1316 |

## Suzuka

| Controller | Gain | Lap time (s) | Avg cross-track error (m) |
|---|---|---:|---:|
| Pure pursuit | `kp=0.1` | 53.20 | 0.0315 |
| Stanley | `kp=2.0` | 53.35 | 0.2218 |
| LQR | default | 53.75 | 0.2072 |

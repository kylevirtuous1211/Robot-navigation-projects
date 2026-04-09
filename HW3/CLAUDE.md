# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Reinforcement learning project for robot path tracking and navigation. Two sub-assignments:

- **HW3-1**: Custom PPO implementation for path tracking with three robot kinematic models (basic, differential drive, bicycle)
- **HW3-2**: Stable-Baselines3 PPO integrated with MLGame3D game framework for navigation with reward shaping

## Commands

```bash
# Dependencies (uses uv with pyproject.toml)
uv sync

# HW3-1: Training (30k iterations, 8 parallel envs, saves to HW3-1/save/model.pt)
cd HW3-1 && python train.py

# HW3-1: Inference with video output
cd HW3-1 && python play.py          # deterministic
cd HW3-1 && python play.py --stoch  # stochastic

# HW3-1: Evaluate over 100 episodes
cd HW3-1 && python eval.py

# HW3-1: Plot training curves
cd HW3-1 && python plot.py

# HW3-2: Training/inference driven by mlgame3d game loop
# rl_play.py (training), model_play.py (inference), kb_play.py (keyboard control)
```

No test suite exists.

## Architecture

### HW3-1: Custom PPO Pipeline

```
train.py → MultiEnv (8 parallel envs via multiprocessing)
         → EnvRunner (collects rollouts, computes GAE)
         → PPO agent (agent.py) updates PolicyNet + ValueNet
```

- **`wrapper.py`** — `PathTrackingEnv` (Gymnasium): generates random cubic spline paths, steps the simulator, computes multi-component rewards. Observation is 14D (past position + orientation + future waypoints), action is 1D continuous.
- **`model.py`** — `PolicyNet` (2 hidden layers, Gaussian output) and `ValueNet` (3 hidden layers). Uses orthogonal initialization and `DiagGaussian` for action sampling.
- **`agent.py`** — PPO with clipped surrogate objective, separate actor/critic optimizers, gradient clipping, LR decay.
- **`multi_env.py`** / **`env_runner.py`** — Multiprocessing environment wrapper and rollout collector with GAE (γ=0.99, λ=0.95).
- **`Simulation/`** — Abstract `Simulator` and `KinematicModel` base classes with three implementations: basic (v + ω), differential drive (left/right wheel speeds), bicycle (acceleration + steering angle). dt=0.1s.

### HW3-2: SB3 + MLGame3D Integration

- **`rl_play.py`** — `MLPlay` class interfaces with mlgame3d game loop. Wraps SB3's PPO with manual rollout buffer management (game loop drives stepping, not `model.learn()`). Custom `RewardManager` with flag capture, distance, and survival reward components.
- **`dummy_env.py`** — Placeholder Gymnasium env for SB3 model initialization (SB3 requires an env at construction, but the real env is the game loop).
- **`model_play.py`** — Loads trained SB3 model for inference within the game framework.

### Key Design Decisions

- HW3-1 is an educational from-scratch PPO; HW3-2 uses production SB3 — they share no code.
- HW3-2 cannot use `model.learn()` because mlgame3d controls the game loop, so rollouts are collected manually and fed to SB3's internal training.
- Three robot types share the same abstract interface (`Simulator` / `KinematicModel`) allowing the environment wrapper to be robot-agnostic.

## Key Parameters

| Parameter | HW3-1 | HW3-2 |
|-----------|-------|-------|
| n_steps | 128 | 2048 |
| batch_size | 64 | 64 |
| epochs | 4 | 10 |
| clip_range | 0.2 | 0.2 |
| γ | 0.99 | 0.99 |
| learning_rate | 1e-4 (decaying) | 3e-4 |
| hidden layers | [64,64] / [64,64,64] | [64,64] |

## Dependencies

Python >=3.11. Core: PyTorch 2.10, stable-baselines3 2.7.1, gymnasium 1.2.3, mlgame3d 0.8.0, numpy <2.0, opencv-python, matplotlib, imageio[ffmpeg], tensorboard.

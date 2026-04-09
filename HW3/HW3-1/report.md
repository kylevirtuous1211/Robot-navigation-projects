# HW3-1 Report: Deep Reinforcement Learning on Path Tracking

## Implementation

### TODO 1 & 2 — Policy & Value Networks (`model.py`)

**PolicyNet**: Two-layer MLP (14 → 64 → 64) with ReLU activations and orthogonal weight initialization. The output passes through a `DiagGaussian` head that maps the 64-dim feature to a 1D action mean, then wraps it in a `FixedNormal` distribution with fixed `std=0.5`. At inference the mode (mean) is used; during training, actions are sampled stochastically.

**ValueNet**: Three-layer MLP (14 → 64 → 64 → 1) with ReLU activations and orthogonal initialization. Outputs a scalar state-value estimate via `state[:, 0]` squeeze.

Both networks use orthogonal initialization with `relu` gain to avoid vanishing/exploding gradients at training start.

### TODO 3 — Rollout Collection (`env_runner.py`)

`EnvRunner.run` collects `n_step=128` steps across `n_env=8` parallel environments:
1. At each step, the current states are fed to `policy_net` to sample actions and log-probabilities, and to `value_net` to get value estimates.
2. Actions are applied to the environment via `env.step(actions)`.
3. After collecting all steps, **Generalized Advantage Estimation (GAE)** is computed with `γ=0.99, λ=0.95` to produce advantage-augmented returns. Returns and advantages are flattened to `(n_step × n_env,)` batches for training.

### TODO 4 — PPO Update (`agent.py`)

The clipped surrogate objective is implemented as:

```
ratio = exp(new_log_prob - old_log_prob)
pg_loss = -min(ratio * A, clip(ratio, 1-ε, 1+ε) * A).mean()
```

The value loss uses clipped value predictions to limit large updates:
```
v_loss = max((returns - V)², (returns - clip(V, V_old±ε))²).mean()
```

Each PPO update runs `4` epochs over the rollout, shuffling and sampling `64`-sample mini-batches. Both actor and critic use Adam optimizers with gradient clipping (`max_norm=0.5`).

### TODO 5 — Training Parameters (`train.py`)

| Parameter | Value |
|-----------|-------|
| n_env | 8 |
| n_step | 128 |
| batch_size | 64 |
| epochs | 4 |
| clip_val (ε) | 0.2 |
| γ | 0.99 |
| λ (GAE) | 0.95 |
| learning rate | 1e-4 (linear decay) |
| n_iter | 30000 |

## Results

The agent learns to track random cubic spline paths using only angular velocity control. Training converges around iteration 15000 with mean episode reward reaching ~150–200 and average episode length ~290 steps (close to the 400-step limit), indicating the agent successfully completes most paths before timeout.

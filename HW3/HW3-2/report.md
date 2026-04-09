# HW3-2 Report: Deep Reinforcement Learning on Proly

## Implementation

### TODO 6 — Reward Design (`rl_play.py`)

The `RewardManager` computes a composite reward each timestep:

```
total = flag_capture + distance + survival + time_penalty + terrain_penalty + circling_penalty
```

#### Flag Capture Reward (`calculate_flag_capture_reward`)
Returns `+200` when `last_checkpoint_index` increases between frames. The large magnitude makes checkpoint capture the dominant learning signal.

#### Distance Reward (`calculate_distance_reward`)
Rewards the agent for moving closer to the next checkpoint:
```
reward = (prev_dist - curr_dist) × 10
```
Skips the frame where a checkpoint is captured to avoid a large negative spike — when a checkpoint is captured, `target_position` jumps to the next (farther) flag, making the distance appear to increase.

#### Survival Reward (`calculate_survival_reward`)
Returns `-100` when `agent_health ≤ 0`, teaching the agent to avoid falling off the island.

#### Auxiliary Penalties
- **Time penalty**: `-0.1` per step — constant pressure to act rather than idle.
- **Terrain penalty**: Penalizes proximity to water (`terrain_type == -1`) and obstacles (`terrain_type == 1`) within 0.8 units of the agent, based on the 5×5 `terrain_grid` observation.
- **Circling penalty**: `-1.0` when the agent displaces less than 1.5 units over 50 consecutive steps, discouraging repetitive looping.

## Training Setup

The agent uses SB3's PPO (`MlpPolicy`, `[64, 64]` Tanh, `n_steps=2048`) integrated with the MLGame3D game loop. Because mlgame3d drives environment stepping externally, rollout data is collected manually via `_add_to_rollout_buffer` and policy updates are triggered via `_update_policy` when the buffer fills. CPU inference is used to avoid timing out at 10× time scale.

The model was trained progressively:
1. Fresh training across all 5 maps (maps 0–4, random per episode)
2. Multi-map training with map shuffling for generalization
3. Focused fine-tuning on map 4 (Pothole Island) with the doubled checkpoint reward

## Results

The trained model reliably captures all 5 checkpoints on maps 1–3 within the 120-second limit. Map 4 (with potholes and irregular coastline) achieves approximately 50% completion rate, limited by the agent's inability to navigate around obstacles when the direct path crosses water.


## Failure analysis (map 4)
Root cause on map 4: Water (terrain_type == -1) appears in the middle of the island as potholes, not just at borders. The agent has learned to avoid water but becomes too cautious 
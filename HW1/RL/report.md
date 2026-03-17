# RL Navigation Implementation Report

## Strategies of Current RL Implementation

The current Reinforcement Learning approach for the robot navigation task is based on the **Proximal Policy Optimization (PPO)** algorithm. 

1. **Continuous Action Space:** The action space is defined as continuous, representing the velocity vector $(v_x, v_y)$ in a 2D space. The agent outputs 2 values clipped between -1 and 1, which are then scaled by `max_velocity` (10.0) to move the robot.
   - **Action Dim (2):** `[v_x, v_y]`
2. **Actor-Critic Architecture:** 
   - **Actor Network:** Predicts the mean and standard deviation of a Gaussian distribution, from which actions are sampled during training. For evaluation, deterministically taking the mean yields more stable and reliable performance.
   - **Critic Network:** A value function approximator that evaluates the quality of the given state ($V(s)$).
3. **Observation Space:** The model observes a 20-dimensional base state vector, stacked over 4 frames for a total of **80 features**:
   - **Polar Goal (2):** `[norm_dist, norm_angle]` — Euclidean distance and relative angle to goal, normalized.
   - **Kinematics (2):** `[vel_x, vel_y]` — Current velocity vector, normalized.
   - **LiDAR (16):** `[lidar_0 ... lidar_15]` — 16 ray casts evenly distributed at 22.5° intervals, normalized to $[0, 1]$.
4. **Reward Design:** The system uses a shaped reward signal combining:
   - **Progress Reward (+):** Potential-based dense reward for decreasing distance to goal.
   - **Safety Clearance Penalty (-):** Continuous penalty when any LiDAR reading is dangerously close to a wall.
   - **Collision Penalty (-):** Heavy terminal penalty on any wall contact.
   - **Goal Reward (+):** Positive reward on reaching the goal threshold.

## Network Architecture Details

1. **The Shared Trunk (`self.shared`):** Instead of building two completely separate neural networks for the Actor and the Critic, the input is passed into a shared feature extractor. It compresses the raw LiDAR, velocity, and goal data into a 64-dimensional "latent representation". Because both the Actor and Critic need to understand the same fundamental geometry (like recognizing obstacles), sharing these layers saves compute and forces the network to learn a unified understanding of the physical space.
2. **The Actor Mean (`self.actor_mean`):** This takes the 64 abstract features and translates them into actual motor commands. It uses a `nn.Tanh()` activation to guarantee outputs are strictly bounded between $[-1, 1]$, allowing for predictable scaling to the robot's physical maximum velocity.
3. **The Actor Standard Deviation (`self.actor_logstd`):** Defined as a standalone, learnable tensor (`nn.Parameter`). By making the exploration noise independent of the state features, the network controls its noise globally. This prevents the agent from "panicking" and killing exploration entirely when encountering certain states (like corners), a common stability issue in PPO.
4. **The Critic (`self.critic`):** Takes the same 64 shared features and collapses them into a single value $V(s)$, acting as the mathematical judge for the Actor's decisions.

## Problems Solved During Implementation

- **Advantage Estimation Stabilization:** Replaced basic Monte-Carlo return computation with robust **Generalized Advantage Estimation (GAE)** formulation. Incorporating temporal-difference (TD) residuals through temporal parameter $\lambda$ smooths the variance in expected returns while maintaining bounds on objective bias.
- **Handling Non-termination Bootstrap:** Replay buffer was adjusted to effectively track actual episode terminations (`is_terminals`) separate from artificial sequence boundaries so values at the edge of trajectories can correctly bootstrap (or reset to 0 upon final step).

## PPO Training Refinement

The initial training phase with a simple observation space (relative X, Y coordinates and 16 basic LiDAR rays) demonstrated a failure mode after 100,000 steps. 

### Failure Mode: Wall Bumping
Despite the inclusion of a dense progress reward, the agent eventually learned a suboptimal policy: charging directly toward the goal at maximum velocity and sliding along or bumping aggressively into walls. 
This arose due to an imbalance in the reward structure: the progress reward (+100.0 possible per step) entirely overshadowed the collision penalty (-50.0). Thus, hitting a wall was mathematically viable if it meant getting closer to the goal.

### Fix 1: Reward Restructuring
To correct this, the reward function was aggressively scaled:
- The collision penalty was increased to `-1000.0`.
- A "Wall Proximity Penalty" was enhanced with a `500.0` scaling multiplier, punishing the agent severely if any LiDAR reading dropped below 20% of its max range, enforcing a safe midline trajectory.

### Fix 2: State Space Downsampling and Normalization
To further aid learning, the state space was heavily refined to mirror standard robotics practices:
- **Polar Goal Coordinates:** The Cartesian `[rel_x, rel_y]` goal representation was replaced with `[normalized_distance, normalized_angle]`.
- **Kinematics:** The agent's current velocity `[v_x, v_y]` was appended to the state so it can learn momentum dynamics.
- **LiDAR Normalization:** The 16 raw rays were explicitly bounded between 0.0 and 1.0 (divided by dynamically calculated max diagonal).

### Fix 3: Reward Shaping and Scaling (Critic Stabilization)
The massive unscaled rewards (e.g. `+100.0` for goal, `-1000.0` for collision) caused the Critic network's Mean Squared Error loss to explode. The reward function was reshaped using standard robotics principles:
- **Potential-Based Progress Reward ($r_{progress}$):** Instead of a static distance penalty, the agent is rewarded strictly for the Euclidean distance *difference* between the previous step and the current step, scaled by a small $\alpha = 0.05$.
- **Constant Time Penalty ($r_{step}$):** A consistent ticking-clock penalty of `-0.02` per step to prevent looping behavior.
- **Terminal Collision Penalty ($r_{collision}$):** The massive collision penalty was reduced. It was initially set to `-5.0`, but this proved smaller than the maximum accumulated time penalty (`-0.02 * 500 = -10.0`), incentivizing the agent to intentionally crash immediately to "save points". It was therefore balanced to `-20.0`.
- **Continuous Clearance Penalty ($r_{clearance}$):** Instead of multiplying proximity by a massive 500 factor, a small weight `w=1.0` is applied linearly when the minimum LiDAR reading $l_{min}$ drops below a 20% safety threshold $d_{safe}$, keeping the agent safely away from walls without crashing the loss function.

### Fix 4: Evaluation Inference Bug (Stochastic vs Deterministic)
While training logs showed steady improvement (average rewards climbing towards +40), the intermediate evaluation videos showed the agent immediately crashing. This occurred because the evaluation loop mistakenly reused the stochastic `agent.select_action(state)` sampling method used for training exploration. 
During inference/evaluation, sampling from standard deviation causes jerky, irrational maneuvers. The evaluation loop was modified to strip exploratory noise via `torch.no_grad()` and deterministically utilize the actor network's pure `mean` output, resolving visual crashing anomalies during testing.

### 發現他不會轉彎
### Fix 5: Tuning for Turning Behavior
It was observed that the agent learned to rush straight toward the goal without learning to turn around walls, likely because the time penalty pressured it to move aggressively at all times.
- **Time penalty removed:** Without `r_{step}`, the agent is free to slow down, arc, and navigate carefully without being punished for taking the optimal curved path.
- **Clearance penalty increased (×3.0):** The wall proximity weight was raised from `1.0` to `3.0`. At the previous weight, the clearance penalty over a single step was at most `0.2`, too weak to compete against the progress reward. Now at `3.0`, approaching a wall directly costs up to `-0.6` per step, actively teaching the agent to turn away.

### Fix 6: Frame Stacking (Temporal Memory)
To address the "jittering" behavior where the agent lacks historical context of its own actions, **Frame Stacking** was implemented. 
- The observation space now concatenates the **last 4 observations** into a single 80-dimensional vector.
- This allows the neural network to perceive its own velocity trends and changing LiDAR patterns over time, enabling it to learn to break out of oscillatory cycles.

### Fix 7: Stagnation Penalty
To prevent "survival farming" (where an agent stays still or spins in place to accumulate small positive rewards/avoid penalties), a strict **Stagnation Penalty** was added:
- The environment tracks the agent's position over a sliding **50-step window**.
- If the total Euclidean displacement over those 50 steps is less than **5.0 pixels**, a reward penalty of **-0.5** is applied.
- This forces the agent to physically relocate to avoid a decaying score, pushing it to explore new paths.

## Curriculum Diversity: Multi-Map and Random Spawning

To prevent the agent from memorizing a single fixed path and improve generalization, the training environment was extended to support randomized episode configurations.

### Multi-Map Training
Three distinct maps (`map1`, `map2`, `map3`) with different obstacle layouts are pre-loaded at initialization. At the start of every episode reset, one map is **randomly selected**. This exposes the agent to a variety of corridor shapes and obstacle placements, forcing it to learn a general navigation policy rather than overfitting to the geometry of a single map.

### Random Start and Goal Spawning
Rather than using fixed start and goal coordinates from `info.json`, the environment now:
1. Samples a random **free-space pixel** on the selected map as the start position.
2. Samples a second random free-space pixel at least **100 pixels away** from the start as the goal.

This is implemented by pre-computing the list of all non-obstacle pixels from the occupancy map at load time (`free_pixels = np.argwhere(occupancy_map == 0)`), and drawing from that list during each `reset()`. Evaluation always uses `map1` with its fixed `info.json` coordinates (`random_spawn=False`) to allow reproducible comparison across training runs.

### Fix 9: Waypoint-Following (Guided Rewards)
To overcome the "local minima" problem and the "circular spinning" behavior of pure exploration, the environment was integrated with the **RRT* algorithm**:
- **Dynamic Path Generation**: At the start of every episode, RRT* is run to generate a valid path from the random start to the target goal.
- **Intermediate Waypoints**: The path is broken into nodes (waypoints). The agent receives a **+5.0 reward** for every waypoint it successfully reaches (within a 20-pixel radius).
- **Targeted Perception**: The observation vector is modified to point the agent's polar coordinates `[norm_dist, norm_angle]` toward the **next nearest waypoint** instead of the distant final goal. This effectively transforms the complex navigation task into a simpler "trace the line" task.

### Fix 10: Safety-Aware RRT* (Path Centering)
To ensure the generated waypoints are as safe as possible, the RRT* planner was upgraded with a **Safety-Aware Cost Function**:
- **Distance Transform**: During the pre-loop stage, a distance map is computed using `cv2.distanceTransform`, storing the distance to the nearest obstacle for every pixel in the occupancy map.
- **Clearance Penalty**: The RRT* cost function was modified to include a safety penalty $p = \frac{100.0}{\text{dist\_to\_wall} + 1.0}$.
- **Resulting Behavior**: Nodes near walls become significantly more expensive than nodes in open space. This forces RRT* to discover paths that naturally follow the midline of corridors and hallways, providing the agent with maximum clearance from hazards.

## Current Reward Function

The agent follows an RRT*-generated path by collecting intermediate waypoints. The reward function is now **guided**:

$$R_t = r_{\text{step}} + r_{\text{clearance}} + r_{\text{collision}} + r_{\text{stagnation}} + r_{\text{waypoint}} + r_{\text{goal}}$$

| Term | Expression | Notes |
|---|---|---|
| $r_{\text{step}}$ | $-0.005$ | Gentle ticking clock |
| $r_{\text{clearance}}$ | $-3.0 \cdot (0.2 - l_{\min})\;$ if $l_{\min} < 0.2$ | Penalizes proximity to walls |
| $r_{\text{collision}}$ | $-20.0$ (terminal) | Ends the episode |
| $r_{\text{stagnation}}$ | $-0.5$ if displacement < 10px over 10 steps | Prevents local loops |
| $r_{\text{waypoint}}$ | $+5.0$ | Bonus for reaching each RRT* waypoint |
| $r_{\text{goal}}$ | $+10.0$ (terminal) | Reward on reaching the final goal |

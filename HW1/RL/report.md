# RL Navigation Implementation Report

## Strategies of Current RL Implementation

The current Reinforcement Learning approach for the robot navigation task is based on the **Proximal Policy Optimization (PPO)** algorithm. 

1. **Continuous Action Space:** The action space is defined as continuous, representing the velocity vector $(v_x, v_y)$ in a 2D space, clipping values between -1 and 1, which are then scaled by a maximum velocity factor to map to the environment setup.
2. **Actor-Critic Architecture:** 
   - **Actor Network:** Predicts the mean and standard deviation of a Gaussian distribution, from which actions are sampled during training. For evaluation, deterministically taking the mean yields more stable and reliable performance.
   - **Critic Network:** A value function approximator that evaluates the quality of the given state ($V(s)$).
3. **Observation Space:** The model observes standard robot-centric state elements:
   - Relative normalized vector pointing towards the actual goal location.
   - 16 simulated LiDAR ray casts evenly distributed around the agent giving normalized distances to the nearest obstacle/wall.
4. **Reward Design:** The system uses a shaped reward signal combining:
   - **Progress Reward (+):** Dense reward proportional to the decrease in distance to the goal compared to the previous step.
   - **Distance Penalty (-):** Slight penalty proportional to distance to encourage closing the gap.
   - **Time Penalty (-):** A small static penalty each step to discourage dawdling.
   - **Collision Penalty (-):** Heavy penalty signaling task failure.
   - **Goal Reward (+):** High reward when reached the target threshold.

## Problems Solved During Implementation

- **Advantage Estimation Stabilization:** Replaced basic Monte-Carlo return computation with robust **Generalized Advantage Estimation (GAE)** formulation. Incorporating temporal-difference (TD) residuals through temporal parameter $\lambda$ smooths the variance in expected returns while maintaining bounds on objective bias.
- **Handling Non-termination Bootstrap:** Replay buffer was adjusted to effectively track actual episode terminations (`is_terminals`) separate from artificial sequence boundaries so values at the edge of trajectories can correctly bootstrap (or reset to 0 upon final step).

## Future Directions

- **Environment Dynamics Improvements:** Incorporating inertia and acceleration rather than granting direct velocity control to represent physical scenarios better.
- **Recurrent Policies:** Giving the network a recurrent component (like LSTM/GRU frames) instead of reacting strictly to static point-in-time observations could prevent the agent from getting stuck locally and help track unobserved obstacles across time.
- **Improved Ray Casting:** The simulated lidar uses simplistic stepping; deploying specialized libraries like `pybullet` or replacing naive DDA casting with vector-based intersection calculations would massively speed up environment simulation frames.
- **Transfer Learning against Changing Goals:** Currently, the environment spawns at fixed start and end points. Shifting to randomized goals alongside curriculum learning (starting easy close goals and progressively moving them outwards) would solidify generalization compared to relying solely on memorizing a single path.

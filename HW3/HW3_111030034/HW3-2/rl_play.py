import os
import sys
import time
import imageio
import logging

import numpy as np
import torch
from dummy_env import DummyEnv
from stable_baselines3 import PPO
from stable_baselines3.common.utils import safe_mean

rl_logger = logging.getLogger("rl_debug")
rl_logger.setLevel(logging.DEBUG)
_log_path = os.path.join(os.path.dirname(__file__), "rl_debug.log")
_handler = logging.FileHandler(_log_path, mode="a")
_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S"))
rl_logger.addHandler(_handler)


class RewardManager:
    def __init__(self):
        self.prev_observation = None
        self.observation = None
        self.step_count = 0
        self.log_interval = 100
        self.position_history = []
        self.history_window = 50  # check every 50 steps
        self._death_penalized = False

    def update(self, observation):
        self.prev_observation = self.observation
        self.observation = observation

    def reset(self):
        self.prev_observation = None
        self.observation = None
        self.step_count = 0
        self.position_history = []
        self._death_penalized = False

    def calculate_flag_capture_reward(self):
        """
        [Flag Capture Reward]
        Goal: When a new flag is capture, give a large reward to encourage the agent to move along the correct path.

        Hints:
        1. Compare 'last frame's checkpoint index' (self.prev_observation["last_checkpoint_index"])
           with 'current frame's checkpoint index' (self.observation["last_checkpoint_index"]).
        2. If the current frame's index > the previous frame's index, it means progress was made. Return a positive reward
        3. If there is no change, return 0.0.
        """
        if self.prev_observation is None:
            return 0.0

        curr_checkpoint = self.observation.get("last_checkpoint_index", 0)
        prev_checkpoint = self.prev_observation.get("last_checkpoint_index", 0)

        if curr_checkpoint > prev_checkpoint:
            curr_time = self.observation.get("current_time", 0.0)
            target_pos = self.observation.get("target_position", [0.0, 0.0])
            new_dist = np.linalg.norm(target_pos)
            rl_logger.info(f"[RL CHECKPOINT] cp {prev_checkpoint}->{curr_checkpoint} "
                  f"at step={self.step_count} time={curr_time:.1f}s | "
                  f"new_target_dist={new_dist:.1f}")
            return 200.0
        return 0.0

    def calculate_proximity_bonus(self):
        """Reward being very close to the next checkpoint — encourages committing to capture."""
        target_pos = self.observation.get("target_position", [0.0, 0.0])
        dist = np.linalg.norm(target_pos)
        if dist < 1.0:
            # Sharp bonus that peaks at dist=0: up to +5.0 per step
            return 5.0 * (1.0 - dist)
        return 0.0

    def calculate_distance_reward(self):
        """
        [Distance Reward]
        Goal: Guide the agent to constantly move closer to the target point.

        Hints:
        1. Calculate 'distance to target in the previous frame' (prev_distance) and 'distance to target in the current frame' (current_distance).
           (Hint: use numpy.linalg.norm to calculate the vector length of target_position)
        2. Compare the two:
           - If current_distance < prev_distance (getting closer) -> reward
           - If current_distance > prev_distance (getting farther) -> penalize
        3. If the distance hasn't changed, return 0.0.
        """
        if self.prev_observation is None:
            return 0.0

        # Skip distance reward on checkpoint capture frames — target_position jumps
        # to the next (farther) checkpoint, creating a misleading negative spike
        curr_cp = self.observation.get("last_checkpoint_index", 0)
        prev_cp = self.prev_observation.get("last_checkpoint_index", 0)
        if curr_cp != prev_cp:
            return 0.0

        curr_target_pos = self.observation.get("target_position", [0.0, 0.0, 0.0])
        prev_target_pos = self.prev_observation.get("target_position", [0.0, 0.0, 0.0])

        current_distance = np.linalg.norm(curr_target_pos)
        prev_distance = np.linalg.norm(prev_target_pos)

        reward = (prev_distance - current_distance) * 15.0
        return reward

    def calculate_survival_reward(self):
        """
        [Survival Reward]
        Goal: Teach the agent the importance of survival - avoid jumpping off the cliff

        Hints:
        Check if agent's health(agent_health) reaches 0
        """
        health = self.observation.get("agent_health", 100)
        if health <= 0:
            if not self._death_penalized:
                self._death_penalized = True
                return -100.0
            return 0.0
        self._death_penalized = False
        return 0.0

    def calculate_respawn_penalty(self):
        """Penalize being in respawn state — dying costs time."""
        is_respawning = self.observation.get("is_respawning", False)
        if is_respawning:
            return -5.0
        return 0.0

    def calculate_time_penalty(self):
        return -0.1

    def calculate_circling_penalty(self):
        """Penalize staying in the same area — detects circling/stalling."""
        pos = self.observation.get("agent_position", None)
        if pos is None:
            return 0.0

        self.position_history.append(np.array(pos[:3]))

        if len(self.position_history) < self.history_window:
            return 0.0

        # Compare current position to position N steps ago
        old_pos = self.position_history[-self.history_window]
        displacement = np.linalg.norm(np.array(pos[:3]) - old_pos)

        # Tiered penalty: nearly stationary is much worse than slow circling
        if displacement < 0.5:
            return -8.0
        if displacement < 1.5:
            return -3.0
        return 0.0

    def calculate_terrain_penalty(self):
        """Penalize proximity to water/obstacles using the 5x5 terrain grid."""
        terrain_grid = self.observation.get("terrain_grid", None)
        if terrain_grid is None:
            return 0.0

        penalty = 0.0
        for row in terrain_grid:
            for cell in row:
                terrain_type = cell.get("terrain_type", 0)
                if terrain_type == 0:  # normal
                    continue
                rel_pos = cell.get("relative_position", [0, 0])
                dist = np.linalg.norm(rel_pos)
                # Only penalize very close hazards — directly underfoot
                if dist < 0.8:
                    penalty -= (0.8 - dist) * 1.0
        return penalty

    def calculate_reward(self):
        checkpoint_score = self.calculate_flag_capture_reward()
        distance_score = self.calculate_distance_reward()
        proximity_bonus = self.calculate_proximity_bonus()
        survival_score = self.calculate_survival_reward()
        time_penalty = self.calculate_time_penalty()
        terrain_penalty = self.calculate_terrain_penalty()
        circling_penalty = self.calculate_circling_penalty()
        respawn_penalty = self.calculate_respawn_penalty()

        total_reward = checkpoint_score + distance_score + proximity_bonus + survival_score + time_penalty + terrain_penalty + circling_penalty + respawn_penalty

        self.step_count += 1

        # Log all observation keys on the very first step
        if self.step_count == 1:
            obs = self.observation
            rl_logger.info(f"[RL KEYS] {list(obs.keys())}")

        if self.step_count % self.log_interval == 0:
            obs = self.observation
            pos = obs.get("agent_position", [0, 0, 0])
            target = obs.get("target_position", [0, 0])
            dist = np.linalg.norm(target)
            vel = obs.get("agent_velocity", [0, 0])
            speed = np.linalg.norm(vel)
            fwd = obs.get("agent_forward_direction", [0, 0])
            cp = obs.get("last_checkpoint_index", -1)
            final = obs.get("reached_final_checkpoint", False)
            t = obs.get("current_time", 0.0)
            health = obs.get("agent_health", -1)
            respawn = obs.get("is_respawning", False)

            # Unpassed checkpoints — absolute positions
            unpassed = obs.get("unpassed_checkpoints", [])
            cp_info = ""
            for i, ucp in enumerate(unpassed[:3]):
                ci = ucp.get("checkpoint_index", -1)
                cp_pos = ucp.get("checkpoint_position", [0, 0, 0])
                cp_info += f" cp{ci}=({cp_pos[0]:.1f},{cp_pos[1]:.1f},{cp_pos[2]:.1f})"

            rl_logger.info(
                f"[RL DEBUG step={self.step_count:04d}] cp={cp} final={final} | "
                f"pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) | "
                f"fwd=({fwd[0]:.2f},{fwd[1]:.2f}) | "
                f"target=({target[0]:.1f},{target[1]:.1f}) dist={dist:.1f} | "
                f"speed={speed:.1f} hp={health} respawn={respawn} | "
                f"time={t:.1f}s | "
                f"rew: cp={checkpoint_score:.1f} dist={distance_score:.1f} "
                f"terr={terrain_penalty:.1f} circ={circling_penalty:.1f} "
                f"resp={respawn_penalty:.1f} "
                f"time={time_penalty:.1f} surv={survival_score:.1f} "
                f"total={total_reward:.1f} | "
                f"next_cps:{cp_info}"
            )

        return float(total_reward)


class MLPlay:
    def __init__(self, observation_structure, action_space_info, *args, **kwargs):
        self.reward_manager = RewardManager()

        self.config = {
            "learning_rate": 0.0001,
            "n_steps": 2048,
            "batch_size": 64,
            "n_epochs": 10,
            "clip_range": 0.15,
            "gamma": 0.99,
            "ent_coef": 0.005,
            "vf_coef": 0.5,
            "max_grad_norm": 0.5,
            "device": "cpu",
            "tensorboard_log": os.path.join(os.path.dirname(__file__), "tensorboard"),
            "policy_kwargs": {"net_arch": [64, 64], "activation_fn": torch.nn.Tanh},
        }
        self.dummy_env = DummyEnv(observation_structure, action_space_info)
        self.prev_observation = None
        self.prev_action = None
        self.prev_log_prob = None
        self.prev_value = None
        self.episode_rewards = []
        self.total_steps = 0
        self.episode_count = 1
        self.update_count = 0
        self.start_time = time.strftime("%Y%m%d_%H%M%S")
        self.model_save_dir = os.path.join(os.path.dirname(__file__), "models", self.start_time)
        self.model_path = os.path.join(os.path.dirname(__file__), "model" + ".zip")

        os.makedirs(self.model_save_dir, exist_ok=True)

        self._initialize_model()
        print("PPO initialized in training mode")

        self.video_writer = None
        self.video_out_path = os.path.join(os.path.dirname(__file__), "save", "RL_play.mp4")
        os.makedirs(os.path.join(os.path.dirname(__file__), "save"), exist_ok=True)

    def reset(self):
        if self.episode_rewards:
            total_reward = sum(self.episode_rewards)
            rl_logger.info(
                f"Episode {self.episode_count}: Total Reward = {total_reward:.2f}, Steps = {len(self.episode_rewards)}"
            )
            self.episode_rewards = []

        self._update_policy()

        self.prev_observation = None
        self.prev_action = None
        self.prev_log_prob = None
        self.prev_value = None
        self.episode_count += 1

        self.reward_manager.reset()

    def update(self, raw_observation, done, *args, **kwargs):
        self.reward_manager.update(raw_observation)
        observation = raw_observation["flattened"]

        reward = self.reward_manager.calculate_reward()
        action, log_prob, value = self._predict_action(observation)

        # Video Saving logic
        frame = raw_observation.get("frame")
        if frame is not None:
            if self.video_writer is None:
                self.video_writer = imageio.get_writer(self.video_out_path, fps=30)
            
            self.video_writer.append_data(frame)

        if self.prev_observation is not None:
            self.episode_rewards.append(reward)

            if not self.model.rollout_buffer.full:
                self._add_to_rollout_buffer(
                    obs=self.prev_observation,
                    action=self.prev_action,
                    reward=reward,
                    done=done,
                    value=self.prev_value,
                    log_prob=self.prev_log_prob,
                )
                if self.model.rollout_buffer.full:
                    done_tensor = np.array([done])
                    value_tensor = torch.as_tensor(value).unsqueeze(0) if value.ndim == 0 else torch.as_tensor(value)
                    self.model.rollout_buffer.compute_returns_and_advantage(last_values=value_tensor, dones=done_tensor)

        self.prev_observation = observation
        self.prev_action = action
        self.prev_log_prob = log_prob
        self.prev_value = value
        self.total_steps += 1

        # NOTE: DO NOT MODIFY.
        # Sending additional dummy discrete actions that would not be needed for this assignment
        return action, (0, 0)

    def _initialize_model(self):
        print("Initializing PPO model...")
        if os.path.exists(self.model_path):
            try:
                self.model = PPO.load(self.model_path, env=self.dummy_env, **self.config, verbose=1)
                print(f"Model loaded from {self.model_path}")
            except Exception as e:
                print(f"Error loading model from {self.model_path}: {e}")
                print("Creating new model...")
                self.model = PPO("MlpPolicy", env=self.dummy_env, **self.config, verbose=1)
        else:
            print(f"No pre-trained model found at {self.model_path}. Creating new model...")
            self.model = PPO("MlpPolicy", env=self.dummy_env, **self.config, verbose=1)

        # NOTE: SB3 is not used in the standard way here. Normally model.learn() drives the
        # entire training loop; here, total_timesteps=0 is used only to initialize the
        # TensorBoard logger and internal SB3 state. The actual rollout collection and policy
        # updates are driven manually by mlgame3d's game loop via _add_to_rollout_buffer()
        # and _update_policy(), because mlgame3d controls the environment stepping externally.
        self.model.learn(total_timesteps=0, tb_log_name=f"PPO_{self.start_time}")

    def _save_model(self):
        if self.model is not None:
            self.model.save(self.model_path)
            print(f"Model saved to {self.model_path}")

            update_path = f"{self.model_save_dir}/ppo_model_{self.update_count}.zip"
            self.model.save(update_path)
            print(f"Model saved to {update_path}")

        if self.video_writer is not None:
            self.video_writer.close()
            self.video_writer = None
            print(f"Video saved to {self.video_out_path}")

    def _predict_action(self, obs):
        obs_tensor = torch.as_tensor(obs).unsqueeze(0).to(self.model.device)
        with torch.no_grad():
            action, value, log_prob = self.model.policy(obs_tensor)
        return action.cpu().numpy().flatten(), log_prob.cpu().numpy().flatten(), value.cpu().numpy().flatten()

    def _add_to_rollout_buffer(self, obs, action, reward, done, value, log_prob):
        if not self.model.rollout_buffer.full:
            self.model.rollout_buffer.add(
                obs=torch.as_tensor(obs).unsqueeze(0),
                action=torch.as_tensor(action).unsqueeze(0),
                reward=torch.as_tensor([reward]),
                episode_start=torch.as_tensor([done]),
                value=torch.as_tensor(value).unsqueeze(0) if value.ndim == 0 else torch.as_tensor(value),
                log_prob=torch.as_tensor(log_prob).unsqueeze(0) if log_prob.ndim == 0 else torch.as_tensor(log_prob),
            )

    def _update_policy(self):
        if self.model.rollout_buffer.size() == 0 or not self.model.rollout_buffer.full:
            return

        print(f"Updating PPO policy with {self.model.rollout_buffer.size()} experiences...")

        self.model.num_timesteps += self.model.rollout_buffer.size()
        self.model.train()
        self.update_count += 1

        self.model.logger.record("train/mean_reward", safe_mean(self.model.rollout_buffer.rewards))
        self.model.logger.record("param/n_steps", self.model.n_steps)
        self.model.logger.record("param/batch_size", self.model.batch_size)
        self.model.logger.record("param/n_epochs", self.model.n_epochs)
        self.model.logger.record("param/gamma", self.model.gamma)
        self.model.logger.record("param/gae_lambda", self.model.gae_lambda)
        self.model.logger.record("param/ent_coef", self.model.ent_coef)
        self.model.logger.record("param/vf_coef", self.model.vf_coef)
        self.model.logger.record("param/max_grad_norm", self.model.max_grad_norm)
        self.model._dump_logs(self.update_count)

        self.model.rollout_buffer.reset()
        print("PPO policy updated successfully")

        self._save_model()

import os
import time

import numpy as np
import torch
from dummy_env import DummyEnv
from stable_baselines3 import PPO
from stable_baselines3.common.utils import safe_mean


class RewardManager:
    def __init__(self):
        self.prev_observation = None
        self.observation = None

    def update(self, observation):
        self.prev_observation = self.observation
        self.observation = observation

    def reset(self):
        self.prev_observation = None
        self.observation = None

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
        return 0.0

    def calculate_survival_reward(self):
        """
        [Survival Reward]
        Goal: Teach the agent the importance of survival - avoid jumpping off the cliff

        Hints:
        Check if agent's health(agent_health) reaches 0
        """
        return 0.0

    def calculate_reward(self):
        """
        [Main Update Loop] (executed every frame)
        Goal: Calculate the total score for this instant

        Hints:
        1. Call the reward each reward components:
           Use self.calculate_...() to get scores for each component.

        2. Sum up rewards:
           total_reward = checkpoint_score + distance_score + survival_score + ...

        Return values:
        - total_reward (float): The total score for this frame.
        """
        # TODO 6: Complete the reward function
        return 0.0


class MLPlay:
    def __init__(self, observation_structure, action_space_info, *args, **kwargs):
        self.reward_manager = RewardManager()

        self.config = {
            "learning_rate": 0.0003,
            "n_steps": 2048,
            "batch_size": 64,
            "n_epochs": 10,
            "clip_range": 0.2,
            "gamma": 0.99,
            "ent_coef": 0,
            "vf_coef": 0.5,
            "max_grad_norm": 0.5,
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

    def reset(self):
        if self.episode_rewards:
            total_reward = sum(self.episode_rewards)
            print(
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

    def _predict_action(self, obs):
        obs_tensor = torch.as_tensor(obs).unsqueeze(0)
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

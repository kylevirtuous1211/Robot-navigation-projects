import os
import imageio

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO

from dummy_env import DummyEnv


class MLPlay:
    def __init__(self, observation_structure, action_space_info, *args, **kwargs):
        self.dummy_env = DummyEnv(observation_structure, action_space_info)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_path = os.path.join(current_dir, "model.zip")

        if os.path.exists(self.model_path):
            print(f"Loading model from: {self.model_path}")
            self.model = PPO.load(self.model_path)
        else:
            print(f"ERROR: model.zip not found at {self.model_path}")
            self.model = None

        self.video_writer = None
        self.video_out_path = os.path.join(current_dir, "save", "model_play.mp4")
        os.makedirs(os.path.join(current_dir, "save"), exist_ok=True)

    def reset(self):
        pass

    def update(self, raw_observation, done, info, *args, **kwargs):
        observation = raw_observation.get("flattened")

        if observation is None or self.model is None:
            # NOTE: DO NOT MODIFY.
            # Sending additional dummy discrete actions that would not be needed for this assignment
            return np.zeros(self.dummy_env.action_space.shape), (0, 0)

        action, _ = self.model.predict(observation, deterministic=True)

        # NOTE: DO NOT MODIFY.
        # Sending additional dummy discrete actions that would not be needed for this assignment
        return action, (0, 0)

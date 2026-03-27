import gymnasium as gym
import numpy as np
from gymnasium import spaces


# NOTE: DO NOT MODIFY THIS CLASS
class DummyEnv(gym.Env):
    """
    A placeholder Gymnasium environment used to initialize the PPO model
    before the real game environment is available.

    mlgame3d provides observation and action metadata (structure and shape) at
    startup, before any actual game frames arrive. DummyEnv uses that metadata
    to construct properly-shaped observation and action spaces so that
    `PPO.load()` or `PPO(...)` can be called immediately without waiting for
    a live environment.

    All `reset()` and `step()` calls return zero-filled observations and neutral
    rewards — this environment is never used for actual rollouts or training.
    """

    def __init__(self, observation_structure, action_space_info):
        super().__init__()

        obs_size = self._calculate_observation_size(observation_structure)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(action_space_info.continuous_size,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        dummy_obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        return dummy_obs, {}

    def step(self, action):
        dummy_obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        return dummy_obs, reward, terminated, truncated, info

    def _calculate_observation_size(self, observation_structure):
        total_size = 0

        for item in observation_structure:
            item_type = item.get("type", "")
            item_key = item.get("key", "")

            if item_key == "flattened":
                vector_size = item.get("vector_size", 0)
                return vector_size

            if item_type == "Vector3":
                total_size += 3
            elif item_type == "Vector2":
                total_size += 2
            elif item_type == "float" or item_type == "int" or item_type == "bool":
                total_size += 1
            elif item_type == "Grid":
                grid_size = item.get("grid_size", 0)
                sub_items = item.get("items", [])
                sub_item_size = self._calculate_observation_size(sub_items)
                total_size += sub_item_size * grid_size * grid_size
            elif item_type == "List":
                sub_items = item.get("items", [])
                sub_item_size = self._calculate_observation_size(sub_items)
                sub_item_count = item.get("item_count", 0)

                if sub_item_count > 0:
                    total_size += sub_item_size * sub_item_count
                else:
                    total_size += sub_item_size

        return total_size

import os

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO


class MLPlay:
    def __init__(self, *args, **kwargs):
        pass

    def reset(self):
        pass

    def update(self, raw_observation, done, info, keyboard=set(), *args, **kwargs):
        action = [0, 0]
        if "up" in keyboard:
            action[1] = 1
        elif "down" in keyboard:
            action[1] = -1
        elif "left" in keyboard:
            action[0] = -1
        elif "right" in keyboard:
            action[0] = 1

        if "space" in keyboard:
            # print the observation you wanna check here
            print(raw_observation["terrain_grid"])

        return action, (0, 0)

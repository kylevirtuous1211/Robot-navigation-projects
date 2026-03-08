import gymnasium as gym
from gymnasium import spaces
import numpy as np
import cv2
from pathlib import Path
import json

# Import utilities from the existing project
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from path_planning.primitives import PathNode, PixelCoordinates
from path_planning.planner_utils import world_map_to_occupancy_map, check_collision_free, check_inside_map

class MapEnv(gym.Env):
    def __init__(self, map_name="map1", goal_threshold=20.0, max_steps=500):
        super(MapEnv, self).__init__()
        
        # Load map data
        map_folder = Path(__file__).parent.parent / "data" / str(map_name)
        map_path = map_folder / "map.png"
        self.world_map = cv2.imread(str(map_path))
        self.occupancy_map = world_map_to_occupancy_map(self.world_map)
        
        info_path = map_folder / "info.json"
        with open(info_path, "r") as file:
            info = json.load(file)
        self.start_coords = np.array(info.get("start_coordinates"), dtype=np.float32)
        self.goal_coords = np.array(info.get("goal_coordinates"), dtype=np.float32)
        
        self.goal_threshold = goal_threshold
        self.max_steps = max_steps
        self.current_step = 0
        self.agent_pos = self.start_coords.copy()
        
        # Action space: (dx, dy) velocity clipped to max_step_size
        self.max_velocity = 10.0
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # Observation space: [relative_goal_x, relative_goal_y, 16 lidar rays]
        self.num_lidar_rays = 16
        self.lidar_range = 100.0
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(2 + self.num_lidar_rays,), dtype=np.float32)

    def _get_lidar(self):
        lidar_values = []
        angles = np.linspace(0, 2*np.pi, self.num_lidar_rays, endpoint=False)
        for angle in angles:
            direction = np.array([np.cos(angle), np.sin(angle)])
            dist = self.lidar_range
            # Ray casting for collision (simplified)
            for d in np.linspace(0, self.lidar_range, 10):
                test_pos = self.agent_pos + direction * d
                node = PathNode(PixelCoordinates(test_pos[0], test_pos[1]))
                if not check_inside_map(self.occupancy_map, node) or self.occupancy_map[int(test_pos[1]), int(test_pos[0])] == 1:
                    dist = d
                    break
            lidar_values.append(dist / self.lidar_range)
        return np.array(lidar_values, dtype=np.float32)

    def _get_obs(self):
        rel_goal = (self.goal_coords - self.agent_pos) / max(self.world_map.shape)
        lidar = self._get_lidar()
        return np.concatenate([rel_goal, lidar])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.agent_pos = self.start_coords.copy()
        self.current_step = 0
        self.prev_dist = np.linalg.norm(self.goal_coords - self.agent_pos)
        return self._get_obs(), {}

    def step(self, action):
        self.current_step += 1
        
        # Apply action
        velocity = action * self.max_velocity
        new_pos = self.agent_pos + velocity
        
        # Continuous collision check using bresenham from planner_utils via check_collision_free
        curr_node = PathNode(PixelCoordinates(self.agent_pos[0], self.agent_pos[1]))
        new_node = PathNode(PixelCoordinates(new_pos[0], new_pos[1]))
        
        terminated = False
        truncated = False
        reward = 0.0
        
        # Collision Check
        if not check_inside_map(self.occupancy_map, new_node) or not check_collision_free(self.occupancy_map, curr_node, new_node):
            reward = -1000.0 # Heavy collision penalty (significantly greater than single-step progress)
            terminated = True
        else:
            self.agent_pos = new_pos
            curr_dist = np.linalg.norm(self.goal_coords - self.agent_pos)
            
            # 1. Progress Reward (Dense)
            reward += 10.0 * (self.prev_dist - curr_dist)
            
            # 2. Distance Penalty
            reward -= 0.01 * curr_dist
            
            # 3. Time Penalty
            reward -= 0.1
            
            # 4. Wall Proximity Penalty (Encourage staying in middle)
            lidar_obs = self._get_lidar()
            min_lidar_dist = np.min(lidar_obs)
            # Penalize heavily if the nearest wall is within 20% of the lidar range
            if min_lidar_dist < 0.2:
                reward -= (0.2 - min_lidar_dist) * 500.0  # Increased penalty scale to counter progress reward
            
            self.prev_dist = curr_dist
            
            # Goal Check
            if curr_dist <= self.goal_threshold:
                reward += 100.0
                terminated = True
        
        if self.current_step >= self.max_steps:
            truncated = True
            
        return self._get_obs(), reward, terminated, truncated, {}

    def render(self):
        canvas = self.world_map.copy()
        cv2.circle(canvas, (int(self.agent_pos[0]), int(self.agent_pos[1])), 5, (255, 0, 0), -1)
        cv2.circle(canvas, (int(self.goal_coords[0]), int(self.goal_coords[1])), int(self.goal_threshold), (0, 255, 0), 2)
        return canvas

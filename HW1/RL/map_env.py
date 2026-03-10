import gymnasium as gym
from gymnasium import spaces
import numpy as np
import cv2
from pathlib import Path
import json
from collections import deque

# Import utilities from the existing project
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from path_planning.primitives import PathNode, PixelCoordinates
from path_planning.planner_utils import world_map_to_occupancy_map, check_collision_free, check_inside_map

class MapEnv(gym.Env):
    def __init__(self, map_names=None, goal_threshold=20.0, max_steps=1000, random_spawn=True, frame_stack=4):
        super(MapEnv, self).__init__()
        
        # Default to all 3 maps
        if map_names is None:
            map_names = ["map1", "map2", "map3"]
        
        self.random_spawn = random_spawn
        self.goal_threshold = goal_threshold
        self.max_steps = max_steps
        self.frame_stack = frame_stack
        
        # Pre-load all maps and their free-space coordinate lists
        data_root = Path(__file__).parent.parent / "data"
        self.maps = []
        for name in map_names:
            map_folder = data_root / name
            world_map = cv2.imread(str(map_folder / "map.png"))
            occupancy_map = world_map_to_occupancy_map(world_map)
            
            # Load default start/goal from info.json as fallback
            with open(map_folder / "info.json", "r") as f:
                info = json.load(f)
            default_start = np.array(info.get("start_coordinates"), dtype=np.float32)
            default_goal = np.array(info.get("goal_coordinates"), dtype=np.float32)
            
            # Pre-compute list of all free (non-wall) pixel coordinates for random sampling
            # occupancy_map: 0 = free, 1 = obstacle
            free_pixels = np.argwhere(occupancy_map == 0)  # shape: (N, 2) -> [row, col]
            
            h, w = world_map.shape[:2]
            max_dist = np.linalg.norm([h, w])
            
            self.maps.append({
                "name": name,
                "world_map": world_map,
                "occupancy_map": occupancy_map,
                "free_pixels": free_pixels,
                "default_start": default_start,
                "default_goal": default_goal,
                "max_dist": max_dist,
            })
        
        # Observation space pre-calculation
        # Individual observation: [goal_dist, goal_angle, vel_x, vel_y, 16 lidar rays] = 20 dims
        self.single_obs_dim = 20
        self.num_lidar_rays = 16
        self.lidar_range = 100.0
        
        # Observation space reflects stack: 20 * 4 = 80 dims
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.single_obs_dim * self.frame_stack,), dtype=np.float32
        )
        
        # Action space: (dx, dy) velocity
        self.max_velocity = 10.0
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # Environment buffers
        self.frames = deque(maxlen=self.frame_stack)
        self.pos_history = deque(maxlen=50)
        
        # Track state
        self.current_vel = np.zeros(2, dtype=np.float32)
        self.current_step = 0
        self._load_map(0)
        self.agent_pos = self.start_coords.copy()

    def _load_map(self, idx):
        """Switch the active map to index idx."""
        m = self.maps[idx]
        self.current_map_idx = idx
        self.world_map = m["world_map"]
        self.occupancy_map = m["occupancy_map"]
        self.free_pixels = m["free_pixels"]
        self.default_start = m["default_start"]
        self.default_goal = m["default_goal"]
        self.max_dist = m["max_dist"]
        # Set default start/goal (overridden in reset if random_spawn=True)
        self.start_coords = self.default_start.copy()
        self.goal_coords = self.default_goal.copy()

    def _sample_free_position(self, min_dist_from_other=None, other_pos=None, max_tries=500):
        """Sample a uniformly random free pixel on the current map."""
        for _ in range(max_tries):
            idx = np.random.randint(len(self.free_pixels))
            row, col = self.free_pixels[idx]
            pos = np.array([col, row], dtype=np.float32)  # [x=col, y=row]
            if min_dist_from_other is not None and other_pos is not None:
                if np.linalg.norm(pos - other_pos) < min_dist_from_other:
                    continue
            return pos
        idx = np.random.randint(len(self.free_pixels))
        row, col = self.free_pixels[idx]
        return np.array([col, row], dtype=np.float32)

    def _get_lidar(self):
        lidar_values = []
        angles = np.linspace(0, 2*np.pi, self.num_lidar_rays, endpoint=False)
        for angle in angles:
            direction = np.array([np.cos(angle), np.sin(angle)])
            dist = self.lidar_range
            for d in np.linspace(0, self.lidar_range, 10):
                test_pos = self.agent_pos + direction * d
                node = PathNode(PixelCoordinates(test_pos[0], test_pos[1]))
                if not check_inside_map(self.occupancy_map, node) or self.occupancy_map[int(test_pos[1]), int(test_pos[0])] == 1:
                    dist = d
                    break
            lidar_values.append(dist / self.lidar_range)
        return np.array(lidar_values, dtype=np.float32)

    def _get_single_obs(self):
        # 1. Goal Vector (Polar Coordinates)
        rel_goal = self.goal_coords - self.agent_pos
        dist_to_goal = np.linalg.norm(rel_goal)
        angle_to_goal = np.arctan2(rel_goal[1], rel_goal[0])
        
        norm_dist = min(dist_to_goal / self.max_dist, 1.0)
        norm_angle = angle_to_goal / np.pi  # [-1.0, 1.0]
        
        # 2. Kinematics (Normalized Velocity)
        norm_vel = self.current_vel / self.max_velocity
        
        # 3. LiDAR
        lidar = self._get_lidar()
        
        return np.concatenate([[norm_dist, norm_angle], norm_vel, lidar]).astype(np.float32)

    def _get_stacked_obs(self):
        return np.concatenate(list(self.frames)).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Randomly pick a map each episode
        map_idx = np.random.randint(len(self.maps))
        self._load_map(map_idx)
        
        if self.random_spawn:
            self.start_coords = self._sample_free_position()
            self.goal_coords = self._sample_free_position(
                min_dist_from_other=100.0, other_pos=self.start_coords
            )
        else:
            self.start_coords = self.default_start.copy()
            self.goal_coords = self.default_goal.copy()
        
        self.agent_pos = self.start_coords.copy()
        self.current_vel = np.zeros(2, dtype=np.float32)
        self.current_step = 0
        self.prev_dist = np.linalg.norm(self.goal_coords - self.agent_pos)
        
        # Initialize memory buffers
        self.pos_history.clear()
        self.pos_history.append(self.agent_pos.copy())
        
        self.frames.clear()
        initial_obs = self._get_single_obs()
        for _ in range(self.frame_stack):
            self.frames.append(initial_obs)
            
        return self._get_stacked_obs(), {}

    def step(self, action):
        self.current_step += 1
        
        self.current_vel = action * self.max_velocity
        new_pos = self.agent_pos + self.current_vel
        
        curr_node = PathNode(PixelCoordinates(self.agent_pos[0], self.agent_pos[1]))
        new_node = PathNode(PixelCoordinates(new_pos[0], new_pos[1]))
        
        terminated = False
        truncated = False
        reward = 0.0
        
        # Collision Check
        if not check_inside_map(self.occupancy_map, new_node) or not check_collision_free(self.occupancy_map, curr_node, new_node):
            reward = -20.0  # Terminal collision penalty
            terminated = True
        else:
            self.agent_pos = new_pos
            self.pos_history.append(self.agent_pos.copy())
            curr_dist = np.linalg.norm(self.goal_coords - self.agent_pos)
            
            # 1. Progress Reward (Potential-based)
            alpha = 0.05
            reward += alpha * (self.prev_dist - curr_dist)
            
            # 2. Safety Clearance Penalty
            lidar_obs = self._get_lidar()
            min_lidar_dist = np.min(lidar_obs)
            safe_dist = 0.2
            if min_lidar_dist < safe_dist:
                reward -= 3.0 * (safe_dist - min_lidar_dist)
            
            # 3. Stagnation Penalty
            # If we have 50 steps of history, check displacement
            if len(self.pos_history) == 50:
                displacement = np.linalg.norm(self.agent_pos - self.pos_history[0])
                if displacement < 5.0: # If moved less than 5 pixels in 50 steps
                    reward -= 0.5 # Apply stagnation penalty
            
            self.prev_dist = curr_dist
            
            # Goal Check
            if curr_dist <= self.goal_threshold:
                reward += 10.0
                terminated = True
        
        if self.current_step >= self.max_steps:
            truncated = True
            
        # Update frame stack
        self.frames.append(self._get_single_obs())
            
        return self._get_stacked_obs(), reward, terminated, truncated, {}

    def render(self):
        canvas = self.world_map.copy()
        cv2.circle(canvas, (int(self.agent_pos[0]), int(self.agent_pos[1])), 6, (255, 50, 50), -1)
        cv2.circle(canvas, (int(self.goal_coords[0]), int(self.goal_coords[1])), int(self.goal_threshold), (50, 220, 50), 2)
        cv2.circle(canvas, (int(self.start_coords[0]), int(self.start_coords[1])), 6, (200, 50, 200), -1)
        cv2.putText(canvas, self.maps[self.current_map_idx]["name"], (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        return canvas

"""
GPU-vectorized path tracking environment.
Manages N parallel environments as batched PyTorch tensor operations on CUDA.
"""
import threading

import numpy as np
import torch

import cubic_spline

MAX_PATH_LEN = 200


def gen_random_path():
    """Generate a single random path on CPU using existing cubic spline code.
    Returns numpy array (L, 4) with columns [x, y, yaw_deg, curvature]."""
    path = [
        [200 + np.random.randint(-30, 30), 50 + np.random.randint(-30, 30)],
        [200 + np.random.randint(-30, 30), 200 + np.random.randint(-30, 30)],
        [150 + np.random.randint(-30, 30), 250 + np.random.randint(-30, 30)],
        [50 + np.random.randint(-30, 30), 350 + np.random.randint(-30, 30)],
    ]
    if np.random.random() > 0.5:
        for i in range(4):
            path[i][0] = 400 - path[i][0]
    path_smooth = cubic_spline.cubic_spline_2d(path, interval=3)
    return np.array(path_smooth, dtype=np.float32)


class PathPool:
    """Pre-generated pool of paths on GPU. Background thread refreshes periodically."""

    def __init__(self, pool_size, device):
        self.pool_size = pool_size
        self.device = device
        self.paths, self.lens = self._generate_pool()
        self._bg_thread = None
        self._next_data = None

    def _generate_pool(self):
        paths_np = np.zeros((self.pool_size, MAX_PATH_LEN, 4), dtype=np.float32)
        lens_np = np.zeros(self.pool_size, dtype=np.int64)
        for i in range(self.pool_size):
            raw = gen_random_path()
            L = min(len(raw), MAX_PATH_LEN)
            paths_np[i, :L] = raw[:L]
            paths_np[i, L:] = raw[L - 1]  # pad with last point
            lens_np[i] = L
        return (
            torch.from_numpy(paths_np).to(self.device),
            torch.from_numpy(lens_np).to(self.device),
        )

    def _bg_generate(self):
        paths_np = np.zeros((self.pool_size, MAX_PATH_LEN, 4), dtype=np.float32)
        lens_np = np.zeros(self.pool_size, dtype=np.int64)
        for i in range(self.pool_size):
            raw = gen_random_path()
            L = min(len(raw), MAX_PATH_LEN)
            paths_np[i, :L] = raw[:L]
            paths_np[i, L:] = raw[L - 1]
            lens_np[i] = L
        self._next_data = (paths_np, lens_np)

    def maybe_refresh(self, iteration, refresh_interval=500):
        if iteration % refresh_interval == 0 and self._bg_thread is None:
            self._bg_thread = threading.Thread(target=self._bg_generate, daemon=True)
            self._bg_thread.start()
        if self._bg_thread is not None and not self._bg_thread.is_alive():
            paths_np, lens_np = self._next_data
            self.paths = torch.from_numpy(paths_np).to(self.device)
            self.lens = torch.from_numpy(lens_np).to(self.device)
            self._bg_thread = None
            self._next_data = None


class GPUVecEnv:
    """Batched path-tracking environment on GPU."""

    def __init__(self, n_env, path_pool, sim_type="basic", device="cuda",
                 init_range=20, max_step=400, dt=0.1,
                 v_range=20, w_range=60):
        self.n_env = n_env
        self.pool = path_pool
        self.sim_type = sim_type
        self.device = device
        self.init_range = init_range
        self.max_step = max_step
        self.dt = dt
        self.v_range = v_range
        self.w_range = w_range

        # Kinematic state
        self.x = torch.zeros(n_env, device=device)
        self.y = torch.zeros(n_env, device=device)
        self.yaw = torch.zeros(n_env, device=device)
        self.v = torch.zeros(n_env, device=device)
        self.w = torch.zeros(n_env, device=device)

        # Tracking state
        self.last_idx = torch.zeros(n_env, dtype=torch.long, device=device)
        self.n_step = torch.zeros(n_env, dtype=torch.long, device=device)

        # Record: last 2 trajectory points (x, y, yaw_deg)
        self.record = torch.zeros(n_env, 2, 3, device=device)

        # Per-env paths
        self.paths = torch.zeros(n_env, MAX_PATH_LEN, 4, device=device)
        self.path_lens = torch.zeros(n_env, dtype=torch.long, device=device)

        # Batch index helper
        self._batch_idx = torch.arange(n_env, device=device)

    def reset(self):
        """Reset all environments. Returns obs (n_env, 14)."""
        mask = torch.ones(self.n_env, dtype=torch.bool, device=self.device)
        self._reset_envs(mask)

        min_idx, _ = self._search_nearest()
        self.last_idx = min_idx
        obs = self._build_obs(min_idx)
        return obs

    def step(self, action):
        """
        action: (n_env,) tensor, values in [-1, 1]
        Returns: obs (n_env, 14), reward (n_env,), done (n_env,) bool
        """
        action = action.clamp(-1.0, 1.0)

        # Kinematic step
        self._kinematic_step(action)

        # Update record
        self.record[:, 0] = self.record[:, 1]
        self.record[:, 1, 0] = self.x
        self.record[:, 1, 1] = self.y
        self.record[:, 1, 2] = self.yaw

        # Nearest point search
        min_idx, min_dist_sq = self._search_nearest()

        # Reward (before reset)
        old_last_idx = self.last_idx.clone()
        reward = self._compute_reward(min_idx, min_dist_sq, old_last_idx)
        self.last_idx = min_idx
        self.n_step += 1

        # Done (before reset)
        done = self._compute_done(min_idx)

        # Auto-reset done envs, then build obs from post-reset state
        if done.any():
            self._reset_envs(done)
            min_idx_new, _ = self._search_nearest()
            # Only update last_idx for reset envs
            self.last_idx = torch.where(done, min_idx_new, self.last_idx)
            min_idx = torch.where(done, min_idx_new, min_idx)

        obs = self._build_obs(min_idx)
        return obs, reward, done

    def _reset_envs(self, mask):
        """Reset environments indicated by boolean mask."""
        n_reset = mask.sum().item()
        if n_reset == 0:
            return

        # Assign new paths from pool
        pool_idx = torch.randint(0, self.pool.pool_size, (n_reset,), device=self.device)
        self.paths[mask] = self.pool.paths[pool_idx]
        self.path_lens[mask] = self.pool.lens[pool_idx]

        # Random start positions
        self.x[mask] = 200.0 + torch.randint(-self.init_range, self.init_range, (n_reset,),
                                              device=self.device, dtype=torch.float32)
        self.y[mask] = 50.0 + torch.randint(-self.init_range, self.init_range, (n_reset,),
                                             device=self.device, dtype=torch.float32)
        self.yaw[mask] = 90.0
        self.v[mask] = 0.0
        self.w[mask] = 0.0
        self.n_step[mask] = 0

        # Record: both slots = initial position
        init_record = torch.stack([self.x[mask], self.y[mask], self.yaw[mask]], dim=-1)  # (n_reset, 3)
        self.record[mask, 0] = init_record
        self.record[mask, 1] = init_record

    def _kinematic_step(self, action):
        """Batched kinematic model step. action: (n_env,)"""
        if self.sim_type == "basic":
            cmd_v = self.v_range * 0.6  # scalar = 12.0
            cmd_w = self.w_range * action  # (n_env,)

            yaw_rad = torch.deg2rad(self.yaw)
            self.x = self.x + cmd_v * torch.cos(yaw_rad) * self.dt
            self.y = self.y + cmd_v * torch.sin(yaw_rad) * self.dt
            # One-step yaw lag: uses OLD self.w, not cmd_w
            self.yaw = (self.yaw + self.w * self.dt) % 360

            self.v = torch.full_like(self.v, cmd_v)
            self.w = cmd_w
        else:
            raise NotImplementedError(f"sim_type '{self.sim_type}' not yet implemented for GPU")

    def _search_nearest(self):
        """Batched nearest point search.
        Returns: (min_idx (n_env,) long, min_dist_sq (n_env,) float)"""
        pos = torch.stack([self.x, self.y], dim=-1).unsqueeze(1)  # (n_env, 1, 2)
        path_xy = self.paths[:, :, :2]  # (n_env, MAX_PATH_LEN, 2)
        sq_dists = ((path_xy - pos) ** 2).sum(dim=-1)  # (n_env, MAX_PATH_LEN)
        min_dist_sq, min_idx = sq_dists.min(dim=1)
        return min_idx, min_dist_sq

    def _compute_reward(self, min_idx, min_dist_sq, old_last_idx):
        """Batched reward computation. min_dist_sq is squared distance."""
        target_yaw = self.paths[self._batch_idx, min_idx, 2]

        error_yaw = (target_yaw - self.yaw) % 360
        error_yaw = torch.where(error_yaw > 180, 360 - error_yaw, error_yaw)

        idx_diff = min_idx - old_last_idx
        progress_reward = torch.where(
            idx_diff > 0, 0.1,
            torch.where(idx_diff == 0, 0.0, -1.0)
        )

        reward = (0.8 * torch.exp(-0.1 * min_dist_sq)
                  + 0.2 * torch.exp(-0.1 * error_yaw ** 2)
                  + progress_reward)
        return reward

    def _compute_done(self, min_idx):
        """Batched done computation."""
        last_point = self.paths[self._batch_idx, self.path_lens - 1, :2]  # (n_env, 2)
        goal_dist = torch.sqrt((self.x - last_point[:, 0]) ** 2
                               + (self.y - last_point[:, 1]) ** 2)

        at_end = (min_idx >= self.path_lens - 1)
        near_goal = (goal_dist < 10.0)
        timeout = (self.n_step >= self.max_step)
        return at_end | near_goal | timeout

    def _build_obs(self, min_idx):
        """Batched observation construction. Returns (n_env, 14)."""
        # Record path: (n_env, 2, 3) -> normalize and convert yaw
        record_obs = self.record.clone()
        record_obs[:, :, 0] /= 600.0
        record_obs[:, :, 1] /= 600.0
        record_obs[:, :, 2] = torch.deg2rad(record_obs[:, :, 2])
        record_flat = record_obs.reshape(self.n_env, 6)

        # Future path: 4 waypoints at offsets [0, 16, 32, 48]
        offsets = torch.tensor([0, 16, 32, 48], device=self.device)
        future_idx = min_idx.unsqueeze(1) + offsets.unsqueeze(0)  # (n_env, 4)
        max_idx = (self.path_lens - 1).unsqueeze(1)
        future_idx = torch.min(future_idx, max_idx)

        batch_exp = self._batch_idx.unsqueeze(1).expand(-1, 4)
        future_xy = self.paths[batch_exp, future_idx, :2]  # (n_env, 4, 2)
        future_flat = future_xy.reshape(self.n_env, 8) / 600.0

        obs = torch.cat([record_flat, future_flat], dim=1)
        return obs

"""
GPU-native rollout collector with GAE computation.
All buffers stay on GPU — no CPU↔GPU transfers during rollout.
"""
from collections import deque

import torch


def compute_gae_gpu(rewards, values, dones, last_values, last_dones, gamma=0.99, lamb=0.95):
    """
    All inputs are GPU tensors.
    rewards, values: (n_step, n_env)
    dones: (n_step, n_env) bool
    last_values: (n_env,)
    last_dones: (n_env,) bool
    Returns: (n_step, n_env) returns tensor
    """
    n_step = rewards.shape[0]
    advs = torch.zeros_like(rewards)
    last_gae_lam = torch.zeros(rewards.shape[1], device=rewards.device)

    for t in reversed(range(n_step)):
        if t == n_step - 1:
            next_nonterminal = (~last_dones).float()
            next_values = last_values
        else:
            next_nonterminal = (~dones[t + 1]).float()
            next_values = values[t + 1]

        delta = rewards[t] + gamma * next_values * next_nonterminal - values[t]
        last_gae_lam = delta + gamma * lamb * next_nonterminal * last_gae_lam
        advs[t] = last_gae_lam

    return advs + values


class GPUEnvRunner:
    def __init__(self, env, s_dim, a_dim, n_step, gamma, lamb, device):
        self.env = env
        self.n_env = env.n_env
        self.s_dim = s_dim
        self.a_dim = a_dim
        self.n_step = n_step
        self.gamma = gamma
        self.lamb = lamb
        self.device = device

        # Initial reset
        self.states = self.env.reset()
        self.dones = torch.ones(self.n_env, dtype=torch.bool, device=device)

        # Rollout buffers — all on GPU
        self.mb_states = torch.zeros(n_step, self.n_env, s_dim, device=device)
        self.mb_actions = torch.zeros(n_step, self.n_env, a_dim, device=device)
        self.mb_values = torch.zeros(n_step, self.n_env, device=device)
        self.mb_rewards = torch.zeros(n_step, self.n_env, device=device)
        self.mb_a_logps = torch.zeros(n_step, self.n_env, device=device)
        self.mb_dones = torch.zeros(n_step, self.n_env, dtype=torch.bool, device=device)

        # Episode tracking
        self.total_rewards = torch.zeros(self.n_env, device=device)
        self.total_len = torch.zeros(self.n_env, dtype=torch.long, device=device)
        self.reward_buf = deque(maxlen=100)
        self.len_buf = deque(maxlen=100)

    def run(self, policy_net, value_net):
        for step in range(self.n_step):
            self.mb_states[step] = self.states
            self.mb_dones[step] = self.dones

            with torch.no_grad():
                actions, a_logps = policy_net(self.states)
                values = value_net(self.states)

            self.mb_actions[step] = actions
            self.mb_a_logps[step] = a_logps
            self.mb_values[step] = values

            # action is (n_env, 1), env expects (n_env,)
            self.states, rewards, self.dones = self.env.step(actions.squeeze(-1))
            self.mb_rewards[step] = rewards

        with torch.no_grad():
            last_values = value_net(self.states)

        self._record()

        mb_returns = compute_gae_gpu(
            self.mb_rewards, self.mb_values, self.mb_dones,
            last_values, self.dones, self.gamma, self.lamb
        )

        N = self.n_step * self.n_env
        return (
            self.mb_states.reshape(N, -1),
            self.mb_actions.reshape(N, -1),
            self.mb_a_logps.reshape(N),
            self.mb_values.reshape(N),
            mb_returns.reshape(N),
        )

    def _record(self):
        """Track completed episodes for logging."""
        for i in range(self.n_step):
            self.total_rewards += self.mb_rewards[i]
            self.total_len += 1

            done_mask = self.mb_dones[i]
            if done_mask.any():
                done_idx = done_mask.nonzero(as_tuple=True)[0]
                for j in done_idx.cpu().tolist():
                    self.reward_buf.append(self.total_rewards[j].item())
                    self.len_buf.append(self.total_len[j].item())
                self.total_rewards[done_mask] = 0
                self.total_len[done_mask] = 0

    def get_performance(self):
        if len(self.reward_buf) == 0:
            return 0.0, 0.0, 0.0
        rewards = list(self.reward_buf)
        mean_return = sum(rewards) / len(rewards)
        std_return = (sum((r - mean_return) ** 2 for r in rewards) / len(rewards)) ** 0.5
        mean_len = sum(self.len_buf) / len(self.len_buf)
        return mean_return, std_return, mean_len

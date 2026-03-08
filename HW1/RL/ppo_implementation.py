import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import numpy as np

class ActorCritic(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super(ActorCritic, self).__init__()
        
        # Shared feature extractor (optional, but good for common patterns)
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh()
        )
        
        # Actor: Mean and Std for Gaussian policy
        self.actor_mean = nn.Sequential(
            nn.Linear(64, action_dim),
            nn.Tanh() # Actions are in [-1, 1]
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, action_dim))
        
        # Critic: State-Value V(s)
        self.critic = nn.Linear(64, 1)

    def forward(self, obs):
        features = self.shared(obs)
        mean = self.actor_mean(features)
        std = torch.exp(self.actor_logstd).expand_as(mean)
        value = self.critic(features)
        return mean, std, value

class PPOAgent:
    def __init__(self, obs_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2, c1=0.5, c2=0.01):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.c1 = c1
        self.c2 = c2
        
        self.policy = ActorCritic(obs_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = ActorCritic(obs_dim, action_dim)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.mse_loss = nn.MSELoss()

    def select_action(self, obs):
        with torch.no_grad():
            obs = torch.FloatTensor(obs).unsqueeze(0)
            mean, std, value = self.policy_old(obs)
            dist = Normal(mean, std)
            action = dist.sample()
            action_logprob = dist.log_prob(action).sum(dim=-1)
        return action.detach().cpu().numpy().flatten(), action_logprob.item(), value.item()

    def update(self, buffer):
        # Convert list of experiences to tensors
        old_states = torch.FloatTensor(np.array(buffer.states))
        old_actions = torch.FloatTensor(np.array(buffer.actions))
        old_logprobs = torch.FloatTensor(np.array(buffer.logprobs))
        old_values = torch.FloatTensor(np.array(buffer.values))
        rewards = buffer.rewards
        is_terminals = buffer.is_terminals
        
        # Calculate Monte Carlo rewards (Returns)
        returns = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(rewards), reversed(is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            returns.insert(0, discounted_reward)
            
        returns = torch.FloatTensor(returns)
        returns = (returns - returns.mean()) / (returns.std() + 1e-7)
        
        # GAE (Simplified for this tutorial-grade implementation)
        advantages = returns - old_values.detach()
        
        # Optimization loop
        for _ in range(10): # Update policy for K epochs
            mean, std, values = self.policy(old_states)
            dist = Normal(mean, std)
            logprobs = dist.log_prob(old_actions).sum(dim=-1)
            dist_entropy = dist.entropy().sum(dim=-1)
            
            ratios = torch.exp(logprobs - old_logprobs.detach())
            
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            
            # Loss Function
            loss = -torch.min(surr1, surr2) + self.c1 * self.mse_loss(values.squeeze(), returns) - self.c2 * dist_entropy
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
            
        self.policy_old.load_state_dict(self.policy.state_dict())

class ReplayBuffer:
    def __init__(self):
        self.states = []
        self.actions = []
        self.logprobs = []
        self.rewards = []
        self.values = []
        self.is_terminals = []

    def clear(self):
        del self.states[:]
        del self.actions[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.values[:]
        del self.is_terminals[:]

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
    def __init__(self, obs_dim, action_dim, lr=3e-4, gamma=0.99, K_epochs=10, eps_clip=0.2, lam=0.95, c1=0.5, c2=0.01):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.lam = lam
        self.c1 = c1
        self.c2 = c2
        self.K_epochs = K_epochs
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.policy = ActorCritic(obs_dim, action_dim).to(self.device)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = ActorCritic(obs_dim, action_dim).to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.mse_loss = nn.MSELoss()

    def select_action(self, obs):
        with torch.no_grad():
            obs = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
            mean, std, value = self.policy_old(obs)
            dist = Normal(mean, std)
            action = dist.sample()
            action_logprob = dist.log_prob(action).sum(dim=-1)
        return action.detach().cpu().numpy().flatten(), action_logprob.item(), value.item()

    def update(self, buffer):
        # Convert list of experiences to tensors and send to device
        old_states = torch.FloatTensor(np.array(buffer.states)).to(self.device)
        old_actions = torch.FloatTensor(np.array(buffer.actions)).to(self.device)
        old_logprobs = torch.FloatTensor(np.array(buffer.logprobs)).to(self.device)
        old_values = torch.FloatTensor(np.array(buffer.values)).to(self.device)
        rewards = buffer.rewards
        is_terminals = buffer.is_terminals
        
        # Calculate Rewards and Advantages using GAE
        returns = []
        advantages = []
        gae = 0
        
        values = old_values.detach().cpu().numpy()
        next_value = 0 # bootstrapping
        
        for i in reversed(range(len(rewards))):
            mask = 1.0 - is_terminals[i]
            delta = rewards[i] + self.gamma * next_value * mask - values[i]
            gae = delta + self.gamma * self.lam * mask * gae
            advantages.insert(0, gae)
            next_value = values[i]
            
        advantages = torch.FloatTensor(advantages).to(self.device)
        returns = advantages + old_values
        
        # Standardize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-7)
        
        
        # Track losses for logging
        total_actor_loss = 0
        total_critic_loss = 0
        
        # Optimization loop
        for _ in range(self.K_epochs):
            mean, std, current_values = self.policy(old_states)
            dist = Normal(mean, std)
            logprobs = dist.log_prob(old_actions).sum(dim=-1)
            dist_entropy = dist.entropy().sum(dim=-1)
            
            ratios = torch.exp(logprobs - old_logprobs.detach())
            
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            
            actor_loss = -torch.min(surr1, surr2).mean()
            critic_loss = self.c1 * self.mse_loss(current_values.squeeze(), returns)
            
            # Loss Function
            loss = actor_loss + critic_loss - self.c2 * dist_entropy.mean()
            
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
            
            total_actor_loss += actor_loss.item()
            total_critic_loss += critic_loss.item()
            
        self.policy_old.load_state_dict(self.policy.state_dict())
        return total_actor_loss / self.K_epochs, total_critic_loss / self.K_epochs

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

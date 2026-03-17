import os
import torch
import numpy as np
from map_env import MapEnv
from ppo_implementation import PPOAgent, ReplayBuffer
import cv2
import imageio
from datetime import datetime
import csv

def train():
    # Hyperparameters
    map_name = "map1"
    max_training_steps = 10000000
    update_timestep = 2000 # Update policy every n timesteps
    lr = 3e-4
    gamma = 0.99
    eps_clip = 0.2
    action_std = 0.5      # Starting std for action distribution
    action_std_decay_rate = 0.05
    min_action_std = 0.1
    action_std_decay_freq = 25000
    
    # Create unique session ID
    session_id = f"ppo_lr{lr}_gamma{gamma}_clip{eps_clip}_steps{max_training_steps}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs("logs", exist_ok=True)
    csv_filename = f"logs/{session_id}.csv"
    
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestep", "Episode", "Avg_Reward", "Actor_Loss", "Critic_Loss", "Eval_Reward"])
    
    # Init environment (uses all 3 maps with random spawns by default)
    env = MapEnv()
    obs_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    # Init agent
    agent = PPOAgent(obs_dim, action_dim, lr=lr, gamma=gamma, eps_clip=eps_clip)
    
    # Check for existing checkpoints to resume training
    checkpoint_dir = "checkpoint"
    if os.path.exists(checkpoint_dir):
        checkpoints = [os.path.join(checkpoint_dir, f) for f in os.listdir(checkpoint_dir) if f.endswith(".pth")]
        if checkpoints:
            latest_ckpt = max(checkpoints, key=os.path.getmtime)
            print(f"[*] Found existing checkpoint: {latest_ckpt}")
            try:
                agent.policy.load_state_dict(torch.load(latest_ckpt, map_location=agent.device))
                agent.policy_old.load_state_dict(agent.policy.state_dict())
                print(f"[*] Successfully loaded weights! Resuming training...")
            except Exception as e:
                print(f"[!] Failed to load checkpoint: {e}. Starting fresh.")
        else:
            print("[*] No existing checkpoints found. Starting fresh.")
    else:
        print("[*] No checkpoint directory found. Starting fresh.")

    buffer = ReplayBuffer()
    
    time_step = 0
    i_episode = 0
    
    # Logging & Evaluation
    log_freq = 2000
    save_video_freq = 100000
    log_running_reward = 0
    log_running_episodes = 0
    recent_actor_loss = 0.0
    recent_critic_loss = 0.0
    
    # Training Loop
    while time_step <= max_training_steps:
        state, _ = env.reset()
        current_ep_reward = 0
        
        for t in range(1, env.max_steps + 1):
            # Select action
            action, action_logprob, state_val = agent.select_action(state)
            
            # Step environment
            next_state, reward, terminated, truncated, _ = env.step(action)
            
            # Save to buffer
            buffer.states.append(state)
            buffer.actions.append(action)
            buffer.logprobs.append(action_logprob)
            buffer.rewards.append(reward)
            buffer.values.append(state_val)
            buffer.is_terminals.append(terminated)
            
            time_step += 1
            current_ep_reward += reward
            state = next_state
            
            # Update agent
            if time_step % update_timestep == 0:
                recent_actor_loss, recent_critic_loss = agent.update(buffer)
                buffer.clear()
            
            # Periodically save a video and evaluate
            if time_step % save_video_freq == 0:
                print(f"Saving evaluation video at timestep {time_step}...")
                os.makedirs("checkpoint", exist_ok=True)
                temp_path = f"checkpoint/{session_id}_step_{time_step}.pth"
                torch.save(agent.policy.state_dict(), temp_path)
                eval_reward = evaluate(model_path=temp_path, num_episodes=5, video_name=f"visualization/trajectory_step_{time_step}.mp4")
            
            # Log progress
            if time_step % log_freq == 0:
                avg_reward = log_running_reward / log_running_episodes if log_running_episodes > 0 else 0
                eval_reward_log = eval_reward if time_step % save_video_freq == 0 else ""
                
                print(f"Episode: {i_episode} \t Timestep: {time_step} \t Avg Reward: {avg_reward:.2f} \t Actor Loss: {recent_actor_loss:.4f} \t Critic Loss: {recent_critic_loss:.4f}")
                
                # Write to CSV
                with open(csv_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([time_step, i_episode, avg_reward, recent_actor_loss, recent_critic_loss, eval_reward_log])
                
                log_running_reward = 0
                log_running_episodes = 0
            
            # Decay std
            if time_step % action_std_decay_freq == 0:
                agent.policy_old.actor_logstd.data = torch.max(
                    agent.policy_old.actor_logstd.data - action_std_decay_rate,
                    torch.full_like(agent.policy_old.actor_logstd.data, np.log(min_action_std))
                )
                agent.policy.actor_logstd.data = agent.policy_old.actor_logstd.data.clone()

            if terminated or truncated:
                break
        
        log_running_reward += current_ep_reward
        log_running_episodes += 1
        i_episode += 1

    # Save model
    os.makedirs("checkpoint", exist_ok=True)
    save_path = f"checkpoint/{session_id}_final.pth"
    torch.save(agent.policy.state_dict(), save_path)
    print(f"Model saved to {save_path}")

def evaluate(model_path="checkpoint/ppo_map1.pth", num_episodes=5, video_name="visualization/trajectory_eval.mp4"):
    # Evaluation uses a fixed non-random setup to see consistent behavior
    env = MapEnv(map_names=["map1"], random_spawn=False)
    obs_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    agent = PPOAgent(obs_dim, action_dim)
    try:
        agent.policy.load_state_dict(torch.load(model_path))
    except FileNotFoundError:
        print(f"Could not find model at {model_path} for evaluation.")
        return
    agent.policy.eval()
    
    eval_rewards = []
    
    for i in range(num_episodes):
        state, _ = env.reset()
        done = False
        total_reward = 0
        frames = []
        
        while not done:
            # Deterministic action for evaluation (use mean instead of sampling)
            with torch.no_grad():
                obs_tensor = torch.FloatTensor(state).unsqueeze(0).to(agent.device)
                mean, _, _ = agent.policy(obs_tensor)
                action = mean.cpu().numpy().flatten()
                
            state, reward, terminated, truncated, _ = env.step(action)
            total_reward += reward
            done = terminated or truncated
            
            frame = env.render()
            frames.append(frame)
            
        print(f"Evaluation Episode {i+1} Reward: {total_reward}")
        eval_rewards.append(total_reward)
        
        # Save a sample trajectory
        if i == 0:
            os.makedirs("visualization", exist_ok=True)
            writer = imageio.get_writer(video_name, fps=30)
            for f in frames:
                # Convert BGR (OpenCV) to RGB (imageio expects RGB)
                rgb_frame = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
                writer.append_data(rgb_frame)
            writer.close()
            print(f"Saved trajectory video to {video_name}")
            
    avg_eval_reward = sum(eval_rewards) / len(eval_rewards)
    print(f"Average Evaluation Reward over {num_episodes} episodes: {avg_eval_reward:.2f}")
    return avg_eval_reward

if __name__ == "__main__":
    train()
    evaluate()

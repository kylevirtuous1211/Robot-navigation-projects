import os
import torch
import numpy as np
import cv2
import imageio
from map_env import MapEnv
from ppo_implementation import PPOAgent

def evaluate_fixed(model_path="checkpoint/ppo_map1.pth", num_episodes=5, output_video="visualization/trajectory_fixed.mp4"):
    env = MapEnv(map_names=["map1"], random_spawn=False)
    obs_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    # Initialize agent and load model
    agent = PPOAgent(obs_dim, action_dim)
    try:
        agent.policy.load_state_dict(torch.load(model_path))
        print(f"Loaded model from {model_path}")
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    agent.policy.eval()
    
    # Record all frames from all episodes to see the full "exploration"
    all_frames = []
    
    for i in range(num_episodes):
        state, _ = env.reset()
        done = False
        total_reward = 0
        episode_frames = []
        
        while not done:
            with torch.no_grad():
                obs_tensor = torch.FloatTensor(state).unsqueeze(0).to(agent.device)
                mean, _, _ = agent.policy(obs_tensor)
                action = mean.cpu().numpy().flatten()
            
            state, reward, terminated, truncated, _ = env.step(action)
            total_reward += reward
            done = terminated or truncated
            
            frame = env.render()
            # Optional: Add episode text to frame
            cv2.putText(frame, f"Episode: {i+1} Reward: {total_reward:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            episode_frames.append(frame)
            
        print(f"Episode {i+1}: Reward = {total_reward:.2f}, Steps = {len(episode_frames)}")
        all_frames.extend(episode_frames)

    if all_frames:
        os.makedirs("visualization", exist_ok=True)
        
        # Use imageio to write the mp4 file
        writer = imageio.get_writer(output_video, fps=30)
        
        for frame in all_frames:
            # Convert BGR (OpenCV) to RGB (imageio expects RGB)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            writer.append_data(rgb_frame)
            
        writer.close()
        print(f"Saved all evaluation episodes to {output_video} ({len(all_frames)} frames total)")
    else:
        print("No frames to save.")

if __name__ == "__main__":
    evaluate_fixed()

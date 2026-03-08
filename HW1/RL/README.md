# RL Navigation Agent

This directory contains a custom Proximal Policy Optimization (PPO) reinforcement learning implementation for 2D robot navigation.

## Setup

It is highly recommended to isolate the dependencies using Conda.

1. **Create the Conda Environment:**
   Run the following command to create an environment named `rl_nav` with Python 3.10.
   ```bash
   conda create -y -n rl_nav python=3.10
   ```

2. **Activate the Environment:**
   ```bash
   conda activate rl_nav
   ```

3. **Install Dependencies:**
   Ensure you are in the project root (`HW1/RL/` or relative to the `requirements.txt`) and install the exact versions for reproducibility. Note: We use specific versions of Torch, Gymnasium, Numpy and OpenCV.
   ```bash
   pip install torch==2.1.2 gymnasium numpy==1.26.4 opencv-python==4.10.0.84
   ```

## Training the Agent

Training the agent is as simple as running the main training script. By default, it uses `map1` and trains for 100,000 steps.

Ensure your environment is active:
```bash
# If not already active
conda activate rl_nav
```

Run the training script:
```bash
python train_rl.py
```

### Outputs

During training, progress is regularly logged to the console. 
Upon completion, or when interrupted securely, the trained policy model is automatically saved to the newly created `checkpoint/` directory (e.g., `checkpoint/ppo_map1.pth`).

## Evaluating and Visualizing

To evaluate a trained model and generate a trajectory video of its performance:

```bash
python evaluate_rl_fixed.py
```

This script will:
1. Load the model from `checkpoint/ppo_map1.pth`.
2. Run the agent in the environment deterministically.
3. Save the best visualization out of multiple evaluation episodes as an MP4 video into the `visualization/` directory (e.g., `visualization/trajectory_fixed.mp4`).

**Note on Video Corruption:**
If you experience video corruption issues with `train_rl.py`'s built-in evaluation function, prefer to use the standalone `evaluate_rl_fixed.py` script which utilizes robust codec fallbacks tailored for the environment container.

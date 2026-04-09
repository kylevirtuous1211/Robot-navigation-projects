"""
GPU-accelerated PPO training for path tracking.
Drop-in replacement for train.py — same checkpoint format, compatible with play.py/eval.py.

Usage:
    python train_gpu.py --n-env 4096 --n-iter 30000 --save-dir ./save_gpu
"""
import argparse
import os
import time

import numpy as np
import torch

from gpu_agent import GPUAgent
from gpu_env import GPUVecEnv, PathPool
from gpu_runner import GPUEnvRunner
from model import PolicyNet, ValueNet


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=int(time.time()))
    parser.add_argument("--save-dir", type=str, default="./save_gpu")
    parser.add_argument("--n-env", type=int, default=4096)
    parser.add_argument("--n-iter", type=int, default=30000)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--pool-size", type=int, default=2048)
    parser.add_argument("--pool-refresh", type=int, default=500)
    parser.add_argument("--sim-type", type=str, default="basic",
                        choices=["basic", "diff_drive", "bicycle"])
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    if args.device == "cuda":
        torch.cuda.manual_seed(args.seed)

    # Parameters
    n_env = args.n_env
    n_step = 128
    a_std = 0.5
    lamb = 0.95
    gamma = 0.99
    clip_val = 0.2
    lr = 1e-4
    n_iter = args.n_iter
    device = args.device

    s_dim = 14
    a_dim = 1
    mb_size = n_env * n_step
    sample_mb_size = max(64, mb_size // 128)
    sample_n_epoch = 4
    max_grad_norm = 0.5
    disp_step = 20
    save_step = 20
    check_step = 500
    save_dir = args.save_dir

    # Create path pool
    print(f"Generating path pool ({args.pool_size} paths)...", end=" ", flush=True)
    path_pool = PathPool(args.pool_size, device)
    print("Done.")

    # Create GPU vectorized environment
    env = GPUVecEnv(n_env, path_pool, sim_type=args.sim_type, device=device)
    runner = GPUEnvRunner(env, s_dim, a_dim, n_step, gamma, lamb, device)

    # Create model (reuse existing model.py)
    policy_net = PolicyNet(s_dim, a_dim, a_std).to(device)
    value_net = ValueNet(s_dim).to(device)
    agent = GPUAgent(policy_net, value_net, lr, max_grad_norm, clip_val,
                     sample_n_epoch, sample_mb_size, mb_size, device)

    # Load model
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    if os.path.exists(os.path.join(save_dir, "model.pt")):
        print("Loading the model ... ", end="")
        state_dict = torch.load(os.path.join(save_dir, "model.pt"), map_location=device)
        policy_net.load_state_dict(state_dict["PolicyNet"])
        value_net.load_state_dict(state_dict["ValueNet"])
        start_it = state_dict["it"]
        print("Done.")
    else:
        start_it = 0

    # Start training
    t_start = time.time()
    policy_net.train()
    value_net.train()

    print(f"\nTraining: n_env={n_env}, n_step={n_step}, mb_size={mb_size}, "
          f"sample_mb_size={sample_mb_size}, device={device}")
    print(f"{'='*50}\n")

    for it in range(start_it, n_iter):
        # Refresh path pool in background
        path_pool.maybe_refresh(it, args.pool_refresh)

        # Collect rollouts
        with torch.no_grad():
            mb_obs, mb_actions, mb_old_a_logps, mb_values, mb_returns = runner.run(
                policy_net, value_net
            )
            mb_advs = mb_returns - mb_values
            mb_advs = (mb_advs - mb_advs.mean()) / (mb_advs.std() + 1e-6)

        # Train
        pg_loss, v_loss = agent.train(
            mb_obs, mb_actions, mb_values, mb_advs, mb_returns, mb_old_a_logps
        )

        # Print results
        if it % disp_step == 0:
            agent.lr_decay(it, n_iter)
            n_sec = time.time() - t_start
            fps = int((it - start_it) * n_env * n_step / n_sec) if n_sec > 0 else 0
            mean_return, std_return, mean_len = runner.get_performance()

            print(f"[{it:5d} / {n_iter:5d}]")
            print("----------------------------------")
            print(f"Timesteps    = {(it - start_it) * mb_size:d}")
            print(f"Elapsed time = {n_sec:.2f} sec")
            print(f"FPS          = {fps:d}")
            print(f"actor loss   = {pg_loss:.6f}")
            print(f"critic loss  = {v_loss:.6f}")
            print(f"mean return  = {mean_return:.6f}")
            print(f"mean length  = {mean_len:.2f}")
            print()

        # Save model
        if it % save_step == 0:
            print("Saving the model ... ", end="")
            torch.save(
                {"it": it, "PolicyNet": policy_net.state_dict(), "ValueNet": value_net.state_dict()},
                os.path.join(save_dir, "model.pt"),
            )
            print("Done.")
            print()

            if it - start_it >= save_step:
                with open(os.path.join(save_dir, "return.txt"), "a") as file:
                    file.write(f"{it:d},{mean_return:.4f},{std_return:.4f}\n")

        # Save checkpoint
        if it % check_step == 0:
            print("Saving the checkpoint ... ", end="")
            torch.save(
                {"it": it, "PolicyNet": policy_net.state_dict(), "ValueNet": value_net.state_dict()},
                os.path.join(save_dir, f"model-{it:05d}.pt"),
            )
            print("Done.")
            print()


if __name__ == "__main__":
    main()

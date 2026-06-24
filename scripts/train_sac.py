#!/usr/bin/env python3
import os
import sys
import argparse
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor

# Add the follower package folder to path to import Robo404Env
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/follower/follower')))
from robo404_env import Robo404Env

class ONNXPolicy(torch.nn.Module):
    """Wrapper to export the SB3 SAC actor to ONNX with deterministic inference."""
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, obs):
        # We enforce deterministic=True for deployment
        return self.actor(obs, deterministic=True)

def export_to_onnx(model, onnx_path):
    print(f"Exporting model to ONNX: {onnx_path}")
    # Extract actor
    actor = model.policy.actor
    onnx_policy = ONNXPolicy(actor)
    onnx_policy.eval()

    # Create dummy observation input [batch_size, obs_dim]
    dummy_input = torch.randn(1, 3, dtype=torch.float32)

    # Export to ONNX
    torch.onnx.export(
        onnx_policy,
        dummy_input,
        onnx_path,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={"observation": {0: "batch_size"}, "action": {0: "batch_size"}},
        opset_version=11
    )
    print("ONNX export completed successfully.")

def main():
    parser = argparse.ArgumentParser(description="Train SAC on Robo404 line following.")
    parser.add_argument("--steps", type=int, default=50000, help="Total training steps")
    parser.add_argument("--save-dir", type=str, default="./models", help="Directory to save models")
    parser.add_argument("--load-model", type=str, default=None, help="Path to load pre-trained SAC model")
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    
    # Initialize Gymnasium environment
    raw_env = Robo404Env()
    # Wrap in Monitor to track stats
    env = Monitor(raw_env)

    try:
        if args.load_model and os.path.exists(args.load_model):
            print(f"Loading existing model from {args.load_model}")
            model = SAC.load(args.load_model, env=env)
        else:
            print("Initializing new SAC model...")
            model = SAC(
                "MlpPolicy",
                env,
                verbose=1,
                learning_rate=3e-4,
                buffer_size=50000,
                learning_starts=1000,
                batch_size=256,
                tau=0.005,
                gamma=0.99,
                train_freq=1,
                gradient_steps=1,
                tensorboard_log=os.path.join(args.save_dir, "tensorboard")
            )

        print(f"Starting training for {args.steps} steps...")
        model.learn(total_timesteps=args.steps, log_interval=10)
        
        # Save model
        zip_path = os.path.join(args.save_dir, "sac_robo404.zip")
        model.save(zip_path)
        print(f"Saved model to {zip_path}")

        # Export ONNX
        onnx_path = os.path.join(args.save_dir, "sac_robo404.onnx")
        export_to_onnx(model, onnx_path)

    except KeyboardInterrupt:
        print("Training interrupted by user. Saving progress...")
        zip_path = os.path.join(args.save_dir, "sac_robo404_interrupted.zip")
        model.save(zip_path)
        print(f"Saved interrupted model to {zip_path}")
        onnx_path = os.path.join(args.save_dir, "sac_robo404_interrupted.onnx")
        export_to_onnx(model, onnx_path)
    finally:
        print("Closing environment...")
        env.close()

if __name__ == "__main__":
    main()

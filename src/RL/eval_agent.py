#!/usr/bin/env python3
"""
Evaluate a trained SAC checkpoint against the *already-running* Gazebo sim.

Intended use: pause the trainer first so it releases /cmd_vel and the
physics-pause services, run this, then resume the trainer.

    kill -STOP <trainer_pid>
    python3 -m RL.eval_agent --model RL/logs/checkpoints/sac_linetrack_60000_steps.zip --episodes 1
    kill -CONT <trainer_pid>
"""
import os
import sys
import argparse
import numpy as np
from stable_baselines3 import SAC

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
import RL  # noqa: F401  registers LineTrack-v0
from RL.line_track_env import LineTrackEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to .zip checkpoint")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--deterministic", action="store_true", default=True)
    args = ap.parse_args()

    print(f"📦 Loading model: {args.model}")
    model = SAC.load(args.model, device="cpu")

    # Build env directly so we can switch off domain randomisation for a
    # clean look at the learned policy.
    env = LineTrackEnv()
    env.dr_enabled = False

    # The trainer leaves Gazebo paused between steps; make sure physics is
    # running and that we have actually received a camera frame + odom before
    # we start, otherwise the first observations are all zeros.
    env._call(env._unpause)
    print("⏳ Waiting for camera/odom data ...")
    for _ in range(600):                     # up to ~3 s
        env._spin(0.005)
        if env.latest_image is not None:
            break
    if env.latest_image is None:
        print("⚠️  No camera frames received — is the sim publishing /camera/image_raw?")

    for ep in range(args.episodes):
        obs, _ = env.reset()
        done = False
        total_r, steps = 0.0, 0
        lin_speeds, errors = [], []
        while not done:
            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            total_r += reward
            steps += 1
            lin_speeds.append(float(env.latest_linear_speed))
            errors.append(float(obs[0]))
        outcome = "TERMINATED (line lost)" if terminated else "completed max_steps"
        print(
            f"▶ Episode {ep+1}: reward={total_r:8.2f}  steps={steps:4d}  "
            f"{outcome}\n"
            f"   linear.x  mean={np.mean(lin_speeds):.3f}  "
            f"max={np.max(lin_speeds):.3f} m/s\n"
            f"   |error|   mean={np.mean(np.abs(errors)):.3f}  "
            f"max={np.max(np.abs(errors)):.3f}"
        )

    env.close()


if __name__ == "__main__":
    main()

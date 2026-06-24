#!/usr/bin/env python3
"""
Export a trained SAC checkpoint to ONNX for the real-robot rl_follower_node.

The deploy node feeds a 3-dim observation [error, delta_error, linear_speed]
and expects a 2-dim action [linear_raw, angular_raw] in [-1, 1] (the node
does the physical scaling itself).

    cd src
    python3 -m RL.export_onnx \
        --model RL/logs/checkpoints/sac_linetrack_60000_steps.zip \
        --out   ../models/sac_robo404.onnx
"""
import os
import sys
import argparse
import torch
import torch.nn as nn
from stable_baselines3 import SAC

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


class DeterministicActor(nn.Module):
    """Wraps the SAC actor to output the deterministic (mean) action,
    squashed to [-1, 1] with tanh — matching predict(deterministic=True)."""

    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, obs):
        # SB3 SAC actor: latent_pi -> mu ; deterministic action = tanh(mu)
        features = self.actor.extract_features(obs, self.actor.features_extractor)
        latent_pi = self.actor.latent_pi(features)
        mu = self.actor.mu(latent_pi)
        return torch.tanh(mu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    print(f"📦 Loading: {args.model}")
    model = SAC.load(args.model, device="cpu")

    wrapper = DeterministicActor(model.policy.actor).eval()

    obs_dim = model.observation_space.shape[0]
    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    torch.onnx.export(
        wrapper,
        dummy,
        args.out,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=11,
    )
    print(f"✅ Exported ONNX → {args.out}")

    # ── sanity check: torch vs onnxruntime agreement ─────────────────────
    import numpy as np
    import onnxruntime as ort

    test = np.array([[0.3, -0.05, 0.1]], dtype=np.float32)
    with torch.no_grad():
        torch_out = wrapper(torch.from_numpy(test)).numpy()
    sess = ort.InferenceSession(args.out)
    onnx_out = sess.run(None, {"obs": test})[0]
    print(f"   torch action: {torch_out.round(4)}")
    print(f"   onnx  action: {onnx_out.round(4)}")
    diff = np.abs(torch_out - onnx_out).max()
    print(f"   max diff: {diff:.2e}  {'✅ OK' if diff < 1e-4 else '⚠️ MISMATCH'}")


if __name__ == "__main__":
    main()

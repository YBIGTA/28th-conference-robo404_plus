#!/usr/bin/env python3
"""
🏎️  SAC line-following trainer for Robo404+
    Requires a running Gazebo sim (headless is fine):
        ros2 launch robo_description train_sim.launch.py

    Run:
        cd /path/to/28th-conference-robo404_plus
        python3 -m RL.train_agent
"""
import os
import sys
import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
import torch

# ── make sure `import RL` triggers gym.register ─────────────────────────
# This adds the project root to sys.path so `RL.__init__` is importable.
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
import RL  # noqa: F401  ← registers LineTrack-v0

# =========================================================================
# 1. Config (超参数配置)
# =========================================================================
SAC_CONFIG = {
    "learning_rate": 3e-4,
    "buffer_size": 100_000,      # 经验回放池  (内存够就开到 1e6)
    "batch_size": 256,
    "ent_coef": "auto",          # SAC 的灵魂：自动调节策略熵
    "gamma": 0.99,               # 折扣因子
    "tau": 0.005,                # target-net 软更新系数
    "learning_starts": 500,      # 先随机探索 500 步再开始学习
    "train_freq": 1,
    "gradient_steps": 1,
    "policy_kwargs": dict(
        net_arch=[256, 256],     # 两层 256 → 比 128 多一点表达力，仍然很快
    ),
}

TOTAL_TIMESTEPS = 200_000
SAVE_FREQ       = 5_000          # 每 5k 步自动存一个 checkpoint
LOG_DIR         = os.path.join(PROJECT_ROOT, "RL", "logs")
TB_DIR          = os.path.join(PROJECT_ROOT, "RL", "sac_tensorboard")
CKPT_DIR        = os.path.join(LOG_DIR, "checkpoints")
FINAL_MODEL     = os.path.join(LOG_DIR, "sac_line_track_final")


def main():
    os.makedirs(CKPT_DIR, exist_ok=True)

    # =====================================================================
    # 2. Env + Agent Setup (环境与智能体)
    # =====================================================================
    # ⚠️  Gazebo 只有一个世界 → 只能创建一个 env 实例。
    #     EvalCallback 需要第二个 env 会抢 /cmd_vel，所以改用 CheckpointCallback。
    env = gym.make("LineTrack-v0")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"🔧 Device: {device}")

    # Resume support: set RESUME_CKPT=/path/to/ckpt.zip to continue training
    # from an existing checkpoint instead of starting fresh. The step counter
    # is preserved so the run picks up where it left off (toward TOTAL_TIMESTEPS).
    resume_ckpt = os.environ.get("RESUME_CKPT", "").strip()
    if resume_ckpt:
        print(f"♻️  Resuming from checkpoint: {resume_ckpt}")
        model = SAC.load(
            resume_ckpt,
            env=env,
            device=device,
            tensorboard_log=TB_DIR,
        )
    else:
        model = SAC(
            "MlpPolicy",
            env,
            verbose=1,
            tensorboard_log=TB_DIR,
            device=device,
            **SAC_CONFIG,
        )

    # =====================================================================
    # 3. Run (训练 + 自动保存)
    # =====================================================================
    checkpoint_cb = CheckpointCallback(
        save_freq=SAVE_FREQ,
        save_path=CKPT_DIR,
        name_prefix="sac_linetrack",
        save_replay_buffer=False,   # 省磁盘
        save_vecnormalize=False,
    )

    print(f"🚀 开始炼丹  total_steps={TOTAL_TIMESTEPS}")
    try:
        model.learn(
            total_timesteps=TOTAL_TIMESTEPS,
            callback=checkpoint_cb,
            log_interval=10,
            # When resuming, keep the existing step counter so the run
            # continues toward TOTAL_TIMESTEPS instead of restarting at 0.
            reset_num_timesteps=not bool(resume_ckpt),
        )
    except KeyboardInterrupt:
        print("\n⏸️  Ctrl-C caught – saving interrupted model …")

    model.save(FINAL_MODEL)
    print(f"✅ 训练完成! 模型已保存 → {FINAL_MODEL}.zip")

    env.close()


if __name__ == "__main__":
    main()
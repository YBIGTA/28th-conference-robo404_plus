"""
RL package – registers the LineTrack-v0 Gymnasium environment.

Usage:
    import RL          # ← this triggers the register() call
    env = gym.make("LineTrack-v0")
"""
import gymnasium as gym

gym.register(
    id="LineTrack-v0",
    entry_point="RL.line_track_env:LineTrackEnv",
    max_episode_steps=500,
)

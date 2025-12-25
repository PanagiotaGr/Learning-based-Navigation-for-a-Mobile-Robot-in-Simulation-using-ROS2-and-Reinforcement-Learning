import os
import time

import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from rl_nav.env import TurtlebotRLEnv


def make_env():
    # factory function για DummyVecEnv
    def _init():
        env = TurtlebotRLEnv()
        return env
    return _init


def main():
    rclpy.init()

    # 1 env για αρχή (μπορείς να το κάνεις και parallel αργότερα)
    vec_env = DummyVecEnv([make_env()])

    # PPO agent
    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        verbose=1,
        n_steps=256,          # πόσα steps πριν update
        batch_size=64,
        gamma=0.99,
        learning_rate=3e-4,
    )

    # Φάκελος για saving
    models_dir = "models_ppo"
    os.makedirs(models_dir, exist_ok=True)

    total_timesteps = 10_000  # ξεκίνα με κάτι μικρό για test

    print(f"Starting training for {total_timesteps} timesteps...")
    start = time.time()

    model.learn(total_timesteps=total_timesteps)

    duration = time.time() - start
    print(f"Training finished in {duration:.1f} seconds.")

    save_path = os.path.join(models_dir, "ppo_turtlebot")
    model.save(save_path)
    print(f"Model saved to {save_path}")

    # Cleanup
    vec_env.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

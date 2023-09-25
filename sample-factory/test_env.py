from env_pendulum_irl import make_env

import numpy as np

env = make_env("my-pendulum-irl")

# Try running the environment for a few steps (stepper should move some)
obs, info = env.reset()
obs_str = ", ".join([f"{val:.2f}" for val in obs])
if info["error"]:
    print("Stopping")
else:
    print(f"{'Step': ^8} | {'Observation': ^36} | {'Reward': ^8} | {'Done': ^8} | Info")
    print(f"{'Reset': ^8} | {obs_str: <36} | {0.0: <8} | {str(False): ^8} | {info}")
    for i in range(10):
        obs, reward, terminated, truncated, info = env.step(np.array([[-25]]))
        obs_str = ", ".join([f"{val:.2f}" for val in obs])
        print(f"{i: ^8} | {obs_str: <36} | {reward: <8.2f} | {str(terminated or truncated): ^8} | {info}")
        if info["error"]:
            print("Stopping")
            break
        if terminated or truncated:
            print("Episode done")
            break
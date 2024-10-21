from mujoco_sim.mujoco_gym_env import GymRenderingSpec, MujocoGymEnv

__all__ = [
    "MujocoGymEnv",
    "GymRenderingSpec",
]

from gym.envs.registration import register

register(
    id="ur5ePickCube-v0",
    entry_point="mujoco_sim.envs:ur5ePickCubeGymEnv",
    max_episode_steps=100,
)
register(
    id="ur5ePickCubeVision-v0",
    entry_point="mujoco_sim.envs:ur5ePickCubeGymEnv",
    max_episode_steps=100,
    kwargs={"image_obs": True},
)

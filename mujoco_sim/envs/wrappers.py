import time
from gym import Env, spaces
import gym
import numpy as np
from gym.spaces import Box
import copy
# from mujoco_sim.spacemouse.spacemouse_expert import SpaceMouseExpert
from mujoco_sim.devices.input_utils import input2action  # Relative import for input2action
from mujoco_sim.devices.keyboard import Keyboard  # Relative import from devices.keyboard
from mujoco_sim.devices.spacemouse import SpaceMouse  # Relative import from devices.spacemouse
from mujoco_sim.utils.rotations import quat_2_euler, euler_2_quat

sigmoid = lambda x: 1 / (1 + np.exp(-x))


class FWBWFrontCameraBinaryRewardClassifierWrapper(gym.Wrapper):
    """
    This wrapper uses the front camera images to compute the reward,
    which is not part of the RL policy's observation space. This is used for the
    forward backward reset-free bin picking task, where there are two classifiers,
    one for classifying success + failure for the forward and one for the
    backward task. Here we also use these two classifiers to decide which
    task to transition into next at the end of the episode to maximize the
    learning efficiency.
    """

    def __init__(self, env: Env, fw_reward_classifier_func, bw_reward_classifier_func):
        # check if env.task_id exists
        assert hasattr(env, "task_id"), "fwbw env must have task_idx attribute"
        assert hasattr(env, "task_graph"), "fwbw env must have a task_graph method"

        super().__init__(env)
        self.reward_classifier_funcs = [
            fw_reward_classifier_func,
            bw_reward_classifier_func,
        ]

    def task_graph(self, obs):
        """
        predict the next task to transition into based on the current observation
        if the current task is not successful, stay in the current task
        else transition to the next task
        """
        success = self.compute_reward(obs)
        if success:
            return (self.task_id + 1) % 2
        return self.task_id

    def compute_reward(self, obs):
        reward = self.reward_classifier_funcs[self.task_id](obs).item()
        return (sigmoid(reward) >= 0.5) * 1

    def step(self, action):
        obs, rew, done, truncated, info = self.env.step(action)
        success = self.compute_reward(self.env.get_front_cam_obs())
        rew += success
        done = done or success
        return obs, rew, done, truncated, info


class FrontCameraBinaryRewardClassifierWrapper(gym.Wrapper):
    """
    This wrapper uses the front camera images to compute the reward,
    which is not part of the observation space
    """

    def __init__(self, env: Env, reward_classifier_func):
        super().__init__(env)
        self.reward_classifier_func = reward_classifier_func

    def compute_reward(self, obs):
        if self.reward_classifier_func is not None:
            logit = self.reward_classifier_func(obs).item()
            return (sigmoid(logit) >= 0.5) * 1
        return 0

    def step(self, action):
        obs, rew, done, truncated, info = self.env.step(action)
        success = self.compute_reward(self.env.get_front_cam_obs())
        rew += success
        done = done or success
        return obs, rew, done, truncated, info


class BinaryRewardClassifierWrapper(gym.Wrapper):
    """
    Compute reward with custom binary reward classifier fn
    """

    def __init__(self, env: Env, reward_classifier_func):
        super().__init__(env)
        self.reward_classifier_func = reward_classifier_func

    def compute_reward(self, obs):
        if self.reward_classifier_func is not None:
            logit = self.reward_classifier_func(obs).item()
            return (sigmoid(logit) >= 0.5) * 1
        return 0

    def step(self, action):
        obs, rew, done, truncated, info = self.env.step(action)
        success = self.compute_reward(obs)
        rew += success
        done = done or success
        return obs, rew, done, truncated, info


class ZOnlyWrapper(gym.ObservationWrapper):
    """
    Removal of X and Y coordinates
    """

    def __init__(self, env: Env):
        super().__init__(env)
        self.observation_space["state"] = spaces.Box(-np.inf, np.inf, shape=(14,))

    def observation(self, observation):
        observation["state"] = np.concatenate(
            (
                observation["state"][:4],
                np.array(observation["state"][6])[..., None],
                observation["state"][10:],
            ),
            axis=-1,
        )
        return observation

import gym
from gym.spaces import flatten_space, flatten


class ObsWrapper(gym.ObservationWrapper):
    """
    This observation wrapper treats the observation space as a dictionary
    of a flattened state space and optionally the images, if available.
    """

    def __init__(self, env):
        super().__init__(env)
        # Flatten the state observation space
        state_space = flatten_space(self.env.observation_space["state"])

        if "images" in self.env.observation_space:
            # If images are part of the observation space
            self.observation_space = gym.spaces.Dict(
                {
                    "state": state_space,
                    "images": self.env.observation_space["images"],
                }
            )
            self.include_images = True
        else:
            # If no image observations
            self.observation_space = gym.spaces.Dict(
                {
                    "state": state_space,
                }
            )
            self.include_images = False

    def observation(self, obs):
        # Flatten the state observation
        obs = {
            "state": flatten(self.env.observation_space["state"], obs["state"]),
        }
        # Include images only if available
        if self.include_images:
            obs["images"] = obs["images"]
        return obs

class Quat2EulerWrapper(gym.ObservationWrapper):
    """
    Convert the quaternion representation of the tcp pose to euler angles
    """

    def __init__(self, env: Env):
        super().__init__(env)
        # from xyz + quat to xyz + euler
        self.observation_space["state"]["tcp_pose"] = spaces.Box(
            -np.inf, np.inf, shape=(6,)
        )

    def observation(self, observation):
        # convert tcp pose from quat to euler
        tcp_pose = observation["state"]["tcp_pose"]
        observation["state"]["tcp_pose"] = np.concatenate(
            (tcp_pose[:3], quat_2_euler(tcp_pose[3:]))
        )
        return observation
class GripperCloseEnv(gym.ActionWrapper):
    """
    Use this wrapper to task that requires the gripper to be closed
    """

    def __init__(self, env):
        super().__init__(env)
        ub = self.env.action_space
        assert ub.shape == (7,)
        self.action_space = Box(ub.low[:6], ub.high[:6])

    def action(self, action: np.ndarray) -> np.ndarray:
        new_action = np.zeros((7,), dtype=np.float32)
        new_action[:6] = action.copy()
        return new_action

    def step(self, action):
        new_action = self.action(action)
        obs, rew, done, truncated, info = self.env.step(new_action)
        if "intervene_action" in info:
            info["intervene_action"] = info["intervene_action"][:6]
        return obs, rew, done, truncated, info


class SpacemouseIntervention(gym.ActionWrapper):
    def __init__(self, env):
        super().__init__(env)

        try:
            # Attempt to initialize the SpaceMouse
            self.expert = SpaceMouse(pos_sensitivity=0.1, rot_sensitivity=0.2)
            self.expert.start_control()
            print("SpaceMouse connected successfully.")
        except OSError:
            # If SpaceMouse is not found, fall back to Keyboard
            print("SpaceMouse not found, falling back to Keyboard.")
            self.expert = Keyboard(pos_sensitivity=0.03, rot_sensitivity=5)

        self.expert.start_control()
        self.last_intervene = 0


    def action(self, action: np.ndarray) -> np.ndarray:
        """
        Input:
        - action: policy action
        Output:
        - action: spacemouse action if nonezero; else, policy action
        """
        action = input2action(self.expert)

        if np.linalg.norm(action) > 0.001:
            self.last_intervene = time.time()
            
        if time.time() - self.last_intervene < 0.5:
            return action, True

        return action, False

    def step(self, action):

        new_action, replaced = self.action(action)

        obs, rew, done, truncated, info = self.env.step(new_action)
        if replaced:
            info["intervene_action"] = new_action

        return obs, rew, done, truncated, info

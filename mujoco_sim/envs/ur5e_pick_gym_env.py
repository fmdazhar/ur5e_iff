from pathlib import Path
from typing import Any, Literal, Tuple, Dict

import gym
import mujoco
import numpy as np
from gym import spaces

try:
    import mujoco_py
except ImportError as e:
    MUJOCO_PY_IMPORT_ERROR = e
else:
    MUJOCO_PY_IMPORT_ERROR = None

from mujoco_sim.controllers import cartesain_motion_controller
from mujoco_sim.mujoco_gym_env import GymRenderingSpec, MujocoGymEnv

_HERE = Path(__file__).parent
_XML_PATH = _HERE / "xmls" / "ur5e_arena.xml"
_UR5E_HOME = np.asarray((-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0)) # UR5e home position
_CARTESIAN_BOUNDS = np.asarray([[0.2, -0.3, 0], [0.6, 0.3, 0.5]])
_SAMPLING_BOUNDS = np.asarray([[0.25, -0.25], [0.55, 0.25]])


class ur5ePickCubeGymEnv(MujocoGymEnv):
    metadata = {"render_modes": ["rgb_array", "human"]}

    def __init__(
        self,
        action_scale: np.ndarray = np.asarray([0.1, 1]),
        seed: int = 0,
        control_dt: float = 0.02,
        physics_dt: float = 0.002,
        time_limit: float = 10.0,
        render_spec: GymRenderingSpec = GymRenderingSpec(),
        render_mode: Literal["rgb_array", "human"] = "rgb_array",
        image_obs: bool = False,
    ):
        self._action_scale = action_scale

        super().__init__(
            xml_path=_XML_PATH,
            seed=seed,
            control_dt=control_dt,
            physics_dt=physics_dt,
            time_limit=time_limit,
            render_spec=render_spec,
        )

        self.metadata = {
            "render_modes": [
                "human",
                "rgb_array",
            ],
            "render_fps": int(np.round(1.0 / self.control_dt)),
        }

        self.render_mode = render_mode
        self.camera_id = (0, 1)
        self.image_obs = image_obs

        # Caching.
        # Caching UR5e joint and actuator IDs.
        self._ur5e_dof_ids = np.asarray([
            self._model.joint("shoulder_pan_joint").id,   # Joint 1: Shoulder pan
            self._model.joint("shoulder_lift_joint").id,  # Joint 2: Shoulder lift
            self._model.joint("elbow_joint").id,          # Joint 3: Elbow
            self._model.joint("wrist_1_joint").id,        # Joint 4: Wrist 1
            self._model.joint("wrist_2_joint").id,        # Joint 5: Wrist 2
            self._model.joint("wrist_3_joint").id         # Joint 6: Wrist 3
        ])
        
        self._ur5e_ctrl_ids = np.asarray([
            self._model.actuator("shoulder_pan").id,      # Actuator for Joint 1
            self._model.actuator("shoulder_lift").id,     # Actuator for Joint 2
            self._model.actuator("elbow").id,             # Actuator for Joint 3
            self._model.actuator("wrist_1").id,           # Actuator for Joint 4
            self._model.actuator("wrist_2").id,           # Actuator for Joint 5
            self._model.actuator("wrist_3").id            # Actuator for Joint 6
        ])
        # body_names = [
        #         # "base",
        #         "shoulder_link",
        #         "upper_arm_link",
        #         "forearm_link",
        #         "wrist_1_link",
        #         "wrist_2_link",
        #         "wrist_3_link",
        #         # "tool0_link",
        #         # "hande",
        #         # "hande_left_finger",
        #         # "hande_right_finger",
        #         ]
        # self.body_ids = [self._model.body(name).id for name in body_names]
           
        #Get the base body ID
        base_body_id = self._model.body("base").id

        # Initialize stack and list to store body IDs for the subtree
        stack = [base_body_id]
        self.body_ids = []

        # Traverse the subtree and collect all body IDs
        while stack:
            body_id = stack.pop()
            self.body_ids.append(body_id)
            
            # Find and add immediate child bodies to the stack
            stack += [
                i
                for i in range(self._model.nbody)
                if self._model.body_parentid[i] == body_id and body_id != i  # Exclude itself
            ]
        print(self.body_ids)   
        
        self._gripper_ctrl_id = self._model.actuator("hande_fingers_actuator").id
        self._pinch_site_id = self._model.site("pinch").id
        # print(self._pinch_site_id)
        self._block_z = self._model.geom("block").size[2]

        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "ur5e/tcp_pos": spaces.Box(
                            -np.inf, np.inf, shape=(3,), dtype=np.float32
                        ),
                        "ur5e/tcp_vel": spaces.Box(
                            -np.inf, np.inf, shape=(3,), dtype=np.float32
                        ),
                        "ur5e/gripper_pos": spaces.Box(
                            -np.inf, np.inf, shape=(1,), dtype=np.float32
                        ),
                        # "ur5e/joint_pos": spaces.Box(-np.inf, np.inf, shape=(7,), dtype=np.float32),
                        # "ur5e/joint_vel": spaces.Box(-np.inf, np.inf, shape=(7,), dtype=np.float32),
                        # "ur5e/joint_torque": specs.Array(shape=(21,), dtype=np.float32),
                        # "ur5e/wrist_force": specs.Array(shape=(3,), dtype=np.float32),
                        "block_pos": spaces.Box(
                            -np.inf, np.inf, shape=(3,), dtype=np.float32
                        ),
                    }
                ),
            }
        )

        if self.image_obs:
            self.observation_space = gym.spaces.Dict(
                {
                    "state": gym.spaces.Dict(
                        {
                            "ur5e/tcp_pos": spaces.Box(
                                -np.inf, np.inf, shape=(3,), dtype=np.float32
                            ),
                            "ur5e/tcp_vel": spaces.Box(
                                -np.inf, np.inf, shape=(3,), dtype=np.float32
                            ),
                            "ur5e/gripper_pos": spaces.Box(
                                -np.inf, np.inf, shape=(1,), dtype=np.float32
                            ),
                        }
                    ),
                    "images": gym.spaces.Dict(
                        {
                            "front": gym.spaces.Box(
                                low=0,
                                high=255,
                                shape=(render_spec.height, render_spec.width, 3),
                                dtype=np.uint8,
                            ),
                            "wrist": gym.spaces.Box(
                                low=0,
                                high=255,
                                shape=(render_spec.height, render_spec.width, 3),
                                dtype=np.uint8,
                            ),
                        }
                    ),
                }
            )

        self.action_space = gym.spaces.Box(
            low=np.asarray([-1.0, -1.0, -1.0, -1.0]),
            high=np.asarray([1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32,
        )

        # NOTE: gymnasium is used here since MujocoRenderer is not available in gym. It
        # is possible to add a similar viewer feature with gym, but that can be a future TODO
        from gymnasium.envs.mujoco.mujoco_rendering import MujocoRenderer

        self._viewer = MujocoRenderer(
            self.model,
            self.data,
            width= render_spec.width, height=render_spec.height,
        )
        self._viewer.render(self.render_mode)

    def reset(
        self, seed=None, **kwargs
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """Reset the environment."""
        mujoco.mj_resetData(self._model, self._data)

        # Reset arm to home position.
        self._data.qpos[self._ur5e_dof_ids] = _UR5E_HOME
        self._data.qvel[self._ur5e_dof_ids] = 0  # Ensure joint velocities are zero

        mujoco.mj_forward(self._model, self._data)

        # Reset mocap body to home position.
        tcp_pos = self._data.sensor("hande/pinch_pos").data
        self._data.mocap_pos[0] = tcp_pos

        # Sample a new block position.
        block_xy = np.random.uniform(*_SAMPLING_BOUNDS)
        self._data.jnt("block").qpos[:3] = (*block_xy, self._block_z)
        mujoco.mj_forward(self._model, self._data)

        # Cache the initial block height.
        self._z_init = self._data.sensor("block_pos").data[2]
        self._z_success = self._z_init + 0.2

        obs = self._compute_observation()
        return obs, {}

    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """
        take a step in the environment.
        Params:
            action: np.ndarray

        Returns:
            observation: dict[str, np.ndarray],
            reward: float,
            done: bool,
            truncated: bool,
            info: dict[str, Any]
        """
        x, y, z, grasp = action

        # Set the mocap position.
        pos = self._data.mocap_pos[0].copy()
        dpos = np.asarray([x, y, z]) * self._action_scale[0]
        npos = np.clip(pos + dpos, *_CARTESIAN_BOUNDS)
        self._data.mocap_pos[0] = npos

        # Set gripper grasp.
        g = self._data.ctrl[self._gripper_ctrl_id] / 255
        dg = grasp * self._action_scale[1]
        ng = np.clip(g + dg, 0.0, 1.0)
        self._data.ctrl[self._gripper_ctrl_id] = ng * 255

        for _ in range(self._n_substeps):
            ctrl = cartesain_motion_controller(
                model=self._model,
                data=self._data,
                site_id=self._pinch_site_id,
                dof_ids=self._ur5e_dof_ids,
                pos=self._data.mocap_pos[0],
                ori=self._data.mocap_quat[0],
            )
            self._data.ctrl[self._ur5e_ctrl_ids] = ctrl

            self._data.qfrc_applied[:] = 0.0
            jac = np.empty((3, self._model.nv))

            subtreeid = 1
            total_mass = self._model.body_subtreemass[subtreeid]
            mujoco.mj_jacSubtreeCom(self._model, self._data, jac, subtreeid)
            self._data.qfrc_applied[:] -=  self._model.opt.gravity * total_mass @ jac

            # for i in self.body_ids:
            #     body_weight = self._model.opt.gravity * self._model.body(i).mass
            #     mujoco.mj_jac(self._model, self._data, jac, None, self._data.body(i).xipos, i)
            #     q_weight = jac.T @ body_weight
            #     self._data.qfrc_applied[:] -= q_weight
       
            # # Integrate joint velocities to obtain joint positions.
            # q = self._data.qpos.copy()
            # self._data.ctrl[self._ur5e_ctrl_ids] = q[self._ur5e_dof_ids]

            mujoco.mj_step(self._model, self._data)
            
        obs = self._compute_observation()
        rew = self._compute_reward()
        terminated = self.time_limit_exceeded()

        return obs, rew, terminated, False, {}

    def render(self):
        rendered_frames = []
        for cam_id in self.camera_id:
            self._viewer.camera = cam_id  # Set the camera based on cam_id
            rendered_frames.append(
                self._viewer.render(render_mode="rgb_array")
            )
        return rendered_frames

    # Helper methods.

    def _compute_observation(self) -> dict:
        obs = {}
        obs["state"] = {}

        tcp_pos = self._data.sensor("hande/pinch_pos").data
        obs["state"]["ur5e/tcp_pos"] = tcp_pos.astype(np.float32)

        tcp_vel = self._data.sensor("hande/pinch_vel").data
        obs["state"]["ur5e/tcp_vel"] = tcp_vel.astype(np.float32)

        gripper_pos = np.array(
            self._data.ctrl[self._gripper_ctrl_id] / 255, dtype=np.float32
        )
        obs["state"]["ur5e/gripper_pos"] = gripper_pos

        # joint_pos = np.stack(
        #     [self._data.sensor(f"ur5e/joint{i}_pos").data for i in range(1, 8)],
        # ).ravel()
        # obs["ur5e/joint_pos"] = joint_pos.astype(np.float32)

        # joint_vel = np.stack(
        #     [self._data.sensor(f"ur5e/joint{i}_vel").data for i in range(1, 8)],
        # ).ravel()
        # obs["ur5e/joint_vel"] = joint_vel.astype(np.float32)

        # joint_torque = np.stack(
        # [self._data.sensor(f"ur5e/joint{i}_torque").data for i in range(1, 8)],
        # ).ravel()
        # obs["ur5e/joint_torque"] = symlog(joint_torque.astype(np.float32))

        # wrist_force = self._data.sensor("ur5e/wrist_force").data.astype(np.float32)
        # obs["ur5e/wrist_force"] = symlog(wrist_force.astype(np.float32))

        if self.image_obs:
            obs["images"] = {}
            obs["images"]["front"], obs["images"]["wrist"] = self.render()
        else:
            block_pos = self._data.sensor("block_pos").data.astype(np.float32)
            obs["state"]["block_pos"] = block_pos

        if self.render_mode == "human":
            self._viewer.render(self.render_mode)

        return obs

    def _compute_reward(self) -> float:
        block_pos = self._data.sensor("block_pos").data
        tcp_pos = self._data.sensor("hande/pinch_pos").data
        dist = np.linalg.norm(block_pos - tcp_pos)
        r_close = np.exp(-20 * dist)
        r_lift = (block_pos[2] - self._z_init) / (self._z_success - self._z_init)
        r_lift = np.clip(r_lift, 0.0, 1.0)
        rew = 0.3 * r_close + 0.7 * r_lift
        return rew


if __name__ == "__main__":
    env = ur5ePickCubeGymEnv(render_mode="human")
    env.reset()
    for i in range(100):
        env.step(np.random.uniform(-1, 1, 4))
        env.render()
    env.close()

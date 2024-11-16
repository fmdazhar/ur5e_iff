from pathlib import Path
from typing import Any, Literal, Tuple, Dict

import gym
import mujoco
import numpy as np
from gym import spaces

from mujoco_sim.controllers import Controller
from mujoco_sim.mujoco_gym_env import GymRenderingSpec, MujocoGymEnv

_HERE = Path(__file__).parent
_XML_PATH = _HERE / "xmls" / "ur5e_arena.xml"
_UR5E_HOME = np.asarray((-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0)) # UR5e home position
_CARTESIAN_BOUNDS = np.asarray([[0.2, -0.3, 0.0], [0.4, 0.3, 0.5]])
_SAMPLING_BOUNDS = np.asarray([[0.25, -0.20], [0.35, 0.20]])


class ur5ePickCubeGymEnv(MujocoGymEnv):
    metadata = {"render_modes": ["rgb_array", "human"]}

    def __init__(
        self,
        action_scale: np.ndarray = np.asarray([0.1, 0.1, 1]),
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
        self._gripper_ctrl_id = self._model.actuator("hande_fingers_actuator").id
        self._pinch_site_id = self._model.site("pinch").id
        self._block_z = self._model.geom("block").size[2]
        
        # Updated identifiers for the geometries and sensors
        self._floor_geom = self._model.geom("floor").id
        self._left_finger_geom = self._model.geom("left_pad1").id
        self._right_finger_geom = self._model.geom("right_pad1").id
        self._hand_geom = self._model.geom("hande_base").id

        self.controller = Controller(
        model=self._model,
        data=self._data,
        site_id=self._pinch_site_id,
        integration_dt= 0.2,
        dof_ids=self._ur5e_dof_ids,
        )

        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "ur5e/tcp_pos": spaces.Box(
                            -np.inf, np.inf, shape=(3,), dtype=np.float32
                        ),
                        "ur5e/tcp_ori": spaces.Box(
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
                        "ur5e/wrist_force": spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
                        "ur5e/wrist_torque": spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
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
                            "ur5e/tcp_ori": spaces.Box(
                                -np.inf, np.inf, shape=(3,), dtype=np.float32
                            ),
                            "ur5e/tcp_vel": spaces.Box(
                                -np.inf, np.inf, shape=(3,), dtype=np.float32
                            ),
                            "ur5e/gripper_pos": spaces.Box(
                                -np.inf, np.inf, shape=(1,), dtype=np.float32
                            ),
                            "ur5e/wrist_force": spaces.Box(
                                -np.inf, np.inf, shape=(3,), dtype=np.float32),
                            "ur5e/wrist_torque": spaces.Box(
                                -np.inf, np.inf, shape=(3,), dtype=np.float32),
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
            low=np.asarray([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]),
            high=np.asarray([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32,
        )


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
        self._success = np.array([0.3, 0.0, 0.3])  # Replace with actual target position if available

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
        x, y, z, qx, qy, qz, grasp = action

        # Set the position.
        pos = self._data.mocap_pos[0].copy()
        dpos = np.asarray([x, y, z]) * self._action_scale[0]
        npos = np.clip(pos + dpos, *_CARTESIAN_BOUNDS)
        self._data.mocap_pos[0] = npos

        new_ori = np.zeros(4)
        quat_des = np.zeros(4)
        ori = self._data.mocap_quat[0].copy()
        nori = np.asarray([qx, qy, qz], dtype=np.float32) * self._action_scale[1]
        mujoco.mju_euler2Quat(new_ori, nori, 'xyz')
        mujoco.mju_mulQuat(quat_des, new_ori, ori)
        self._data.mocap_quat[0] = quat_des

        # Set gripper grasp.
        g = self._data.ctrl[self._gripper_ctrl_id] / 255
        dg = grasp * self._action_scale[2]
        ng = np.clip(g + dg, 0.0, 1.0)
        self._data.ctrl[self._gripper_ctrl_id] = ng * 255

        for _ in range(self._n_substeps):

            ctrl = self.controller.control(
                pos=self._data.mocap_pos[0].copy(),
                ori=self._data.mocap_quat[0].copy(),
            )
            
            # Set the control signal.
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
    
    def _get_contact_info(self, geom1_id: int, geom2_id: int) -> Tuple[float, np.ndarray]:
        """Get distance and normal for contact between geom1 and geom2."""
        # Iterate through contacts
        for contact in self._data.contact[:self._data.ncon]:
            # Check if contact involves geom1 and geom2
            if {contact.geom1, contact.geom2} == {geom1_id, geom2_id}:
                distance = contact.dist
                normal = contact.frame[:3]
                return distance, normal
        return float('inf'), np.zeros(3)  # No contact

    def _compute_observation(self) -> dict:
        obs = {}
        obs["state"] = {}

        tcp_pos = self._data.sensor("hande/pinch_pos").data
        obs["state"]["ur5e/tcp_pos"] = tcp_pos.astype(np.float32)

        tcp_ori_quat = self._data.sensor("hande/pinch_quat").data
        tcp_ori_euler = np.zeros(3)
        mujoco.mju_quat2Vel(tcp_ori_euler, tcp_ori_quat, 1.0)
        obs["state"]["ur5e/tcp_ori"] = tcp_ori_euler.astype(np.float32)

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

        wrist_force = self._data.sensor("ur5e/wrist_force").data.astype(np.float32)
        obs["state"]["ur5e/wrist_force"] = wrist_force.astype(np.float32)

        wrist_torque = self._data.sensor("ur5e/wrist_torque").data.astype(np.float32)
        obs["state"]["ur5e/wrist_torque"] = wrist_torque.astype(np.float32)

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
        block_pos = self._data.sensor("block_pos").data[:3]
        tcp_pos = self._data.sensor("hande/pinch_pos").data
        dist = np.linalg.norm(block_pos - tcp_pos)
            
        # 1. box_target: reward for the block approaching a target position
        box_target = 1 - np.tanh(5 * np.linalg.norm(self._success - block_pos))
        
        # 2. gripper_box: reward for the gripper being close to the block
        gripper_box = 1 - np.tanh(5 * dist)
        
        # 3. no_floor_collision: reward for not colliding with the floor
        left_dist, _ = self._get_contact_info(self._left_finger_geom, self._floor_geom)
        right_dist, _ = self._get_contact_info(self._right_finger_geom, self._floor_geom)
        hand_dist, _ = self._get_contact_info(self._hand_geom, self._floor_geom)
        floor_collision = any(dist < 0 for dist in [left_dist, right_dist, hand_dist])
        no_floor_collision = 1 - floor_collision

        # print(f"box_target: {box_target}, gripper_box: {gripper_box}, no_floor_collision: {no_floor_collision}")
        
        # Combine rewards with scaling
        rew = (
            8.0 * box_target + 
            4.0 * gripper_box + 
            0.25 * no_floor_collision
        )

        return rew

if __name__ == "__main__":
    env = ur5ePickCubeGymEnv(render_mode="human")
    env.reset()
    for i in range(100):
        env.step(np.random.uniform(-1, 1, 7))
        env.render()
    env.close()

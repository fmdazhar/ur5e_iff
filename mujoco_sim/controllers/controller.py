from typing import Optional, Tuple, Union
import mujoco
import numpy as np


class Controller:
    def __init__(
        self,
        model,
        data,
        site_id,
        integration_dt,
        dof_ids: np.ndarray,
    ):
        self.model = model
        self.data = data
        self.site_id = site_id
        self.integration_dt = integration_dt if integration_dt is not None else model.opt.timestep
        self.dof_ids = dof_ids

        # Default parameters
        self.damping_ratio = 0.0
        self.error_tolerance_pos = 0.01
        self.error_tolerance_ori = 0.01
        self.max_pos_error = None
        self.max_ori_error = None
        self.method = "dls"
        self.pos_gains = (1, 1, 1)
        self.ori_gains = (0.5, 0.5, 0.5)
        self.pos_kd = None
        self.ori_kd = None

        # Preallocate memory for commonly used variables
        self.quat = np.zeros(4)
        self.quat_des = np.zeros(4)
        self.quat_conj = np.zeros(4)
        self.quat_err = np.zeros(4)
        self.ori_err = np.zeros(3)
        self.J_v = np.zeros((3, model.nv), dtype=np.float64)
        self.J_w = np.zeros((3, model.nv), dtype=np.float64)
        self.M = np.zeros((model.nv, model.nv), dtype=np.float64)

    def set_parameters(
        self,
        damping_ratio: float = 1,
        error_tolerance_pos: float = 0.01,
        error_tolerance_ori: float = 0.01,
        max_pos_error: Optional[float] = None,
        max_ori_error: Optional[float] = None,
        pos_gains: Union[Tuple[float, float, float], np.ndarray] = (1, 1, 1),
        ori_gains: Union[Tuple[float, float, float], np.ndarray] = (1, 1, 1),
        pos_kd: Optional[Union[Tuple[float, float, float], np.ndarray]] = None,
        ori_kd: Optional[Union[Tuple[float, float, float], np.ndarray]] = None,
        method: str = "dls",
    ):
        self.damping_ratio = damping_ratio
        self.error_tolerance_pos = error_tolerance_pos
        self.error_tolerance_ori = error_tolerance_ori
        self.max_pos_error = max_pos_error
        self.max_ori_error = max_ori_error
        self.pos_gains = pos_gains
        self.ori_gains = ori_gains
        self.pos_kd = pos_kd
        self.ori_kd = ori_kd
        if method in ["dynamics", "pinv", "svd", "trans", "dls"]:
            self.method = method
        else:
            raise ValueError("Method must be one of 'dynamics', 'pinv', 'svd', 'trans', 'dls'")

    def compute_gains(self, gains, kd_values, method: str):
        kp = np.asarray(gains)
        if method == "dynamics":
            if kd_values is None:
                kd = self.damping_ratio * 2 * np.sqrt(kp)
            else:
                kd = np.asarray(kd_values)
        else:
            if kd_values is None:
                kd = self.damping_ratio * kp
            else:
                kd = np.asarray(kd_values) / self.integration_dt

        return np.stack([kp, kd], axis=-1)

    def control(self, pos: Optional[np.ndarray] = None, ori: Optional[np.ndarray] = None) -> np.ndarray:
        # Desired position and orientation
        x_des = self.data.site_xpos[self.site_id] if pos is None else np.asarray(pos)
        if ori is None:
            mujoco.mju_mat2Quat(self.quat_des, self.data.site_xmat[self.site_id])
        else:
            ori = np.asarray(ori)
            if ori.shape == (3, 3):
                mujoco.mju_mat2Quat(self.quat_des, ori)
            else:
                self.quat_des[:] = ori

        kp_kv_pos = self.compute_gains(self.pos_gains, self.pos_kd, self.method)
        kp_kv_ori = self.compute_gains(self.ori_gains, self.ori_kd, self.method)

        ddx_max = self.max_pos_error if self.max_pos_error is not None else 0.0
        dw_max = self.max_ori_error if self.max_ori_error is not None else 0.0

        q = self.data.qpos[self.dof_ids]
        dq = self.data.qvel[self.dof_ids]

        mujoco.mj_jacSite(self.model, self.data, self.J_v, self.J_w, self.site_id)
        J_v = self.J_v[:, self.dof_ids]
        J_w = self.J_w[:, self.dof_ids]
        J = np.concatenate([J_v, J_w], axis=0)

        # Position Control
        x_err = self.data.site_xpos[self.site_id] - x_des
        dx_err = J_v @ dq

        x_err_norm = np.linalg.norm(x_err)
        if x_err_norm < self.error_tolerance_pos:
            x_err.fill(0)
            dx_err.fill(0)

        x_err *= -kp_kv_pos[:, 0]
        dx_err *= -kp_kv_pos[:, 1]

        if ddx_max > 0.0:
            x_err_sq_norm = np.sum(x_err**2)
            ddx_max_sq = ddx_max**2
            if x_err_sq_norm > ddx_max_sq:
                x_err *= ddx_max / np.sqrt(x_err_sq_norm)

        ddx = x_err + dx_err

        # Orientation Control
        mujoco.mju_mat2Quat(self.quat, self.data.site_xmat[self.site_id])
        mujoco.mju_negQuat(self.quat_conj, self.quat_des)
        mujoco.mju_mulQuat(self.quat_err, self.quat, self.quat_conj)
        mujoco.mju_quat2Vel(self.ori_err, self.quat_err, 1.0)
        w_err = J_w @ dq

        ori_err_norm = np.linalg.norm(self.ori_err)
        if ori_err_norm < self.error_tolerance_ori:
            self.ori_err.fill(0)
            w_err.fill(0)

        self.ori_err *= -kp_kv_ori[:, 0]
        w_err *= -kp_kv_ori[:, 1]

        if dw_max > 0.0:
            ori_err_sq_norm = np.sum(self.ori_err**2)
            dw_max_sq = dw_max**2
            if ori_err_sq_norm > dw_max_sq:
                self.ori_err *= dw_max / np.sqrt(ori_err_sq_norm)

        dw = self.ori_err + w_err

        error = np.concatenate([ddx, dw], axis=0)

        if self.method == "dynamics":
            mujoco.mj_fullM(self.model, self.M, self.data.qM)
            M = self.M[self.dof_ids, :][:, self.dof_ids]
            M_inv = np.linalg.inv(M)
            ddq = M_inv @ J.T @ error
            dq += ddq * self.integration_dt
            q += dq * self.integration_dt

        elif self.method == "pinv":
            J_pinv = np.linalg.pinv(J)
            dq = J_pinv @ error
            q += dq
        elif self.method == "svd":
            U, S, Vt = np.linalg.svd(J, full_matrices=False)
            S_inv = np.zeros_like(S)
            S_inv[S > 1e-5] = 1.0 / S[S > 1e-5]
            J_pinv = Vt.T @ np.diag(S_inv) @ U.T
            dq = J_pinv @ error
            q += dq
        elif self.method == "trans":
            dq = J.T @ error
            q += dq
        else:
            damping = 1e-4
            lambda_I = damping * np.eye(J.shape[0])
            dq = J.T @ np.linalg.inv(J @ J.T + lambda_I) @ error
            q += dq

        q_min = self.model.actuator_ctrlrange[:6, 0]
        q_max = self.model.actuator_ctrlrange[:6, 1]
        np.clip(q, q_min, q_max, out=q)

        return q

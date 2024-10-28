from typing import Optional, Tuple, Union
import mujoco
import numpy as np
from dm_robotics.transformations import transformations as tr


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
        self.damping_ratio = 1.0
        self.error_tolerance_pos = 0.01
        self.error_tolerance_ori = 0.01
        self.max_pos_error = None
        self.max_ori_error = None
        self.method = "dynamics"
        self.pos_gains = (0.5, 0.5, 0.5)
        self.ori_gains = (0.25, 0.25, 0.25)
        self.pos_kd = None
        self.ori_kd = None

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

    def pd_control(self, x, x_des, dx, kp_kv, ddx_max=0.0) -> np.ndarray:
        x_err = x - x_des
        dx_err = dx

        x_err_norm = np.linalg.norm(x_err)
        if x_err_norm < self.error_tolerance_pos:
            x_err = np.zeros_like(x_err)
            dx_err = np.zeros_like(dx_err)

        x_err *= -kp_kv[:, 0]
        dx_err *= -kp_kv[:, 1]

        # Print the position error and velocity error for debugging
        print("Position Error (x_err):", x_err)
        print("Velocity Error (dx_err):", dx_err)

        if ddx_max > 0.0:
            x_err_sq_norm = np.sum(x_err**2)
            ddx_max_sq = ddx_max**2
            if x_err_sq_norm > ddx_max_sq:
                x_err *= ddx_max / np.sqrt(x_err_sq_norm)

        return x_err + dx_err

    def pd_control_orientation(self, quat, quat_des, w, kp_kv, dw_max=0.0) -> np.ndarray:
        quat_err = tr.quat_diff_active(source_quat=quat_des, target_quat=quat)
        ori_err = tr.quat_to_axisangle(quat_err)
        w_err = w

        ori_err_norm = np.linalg.norm(ori_err)
        if ori_err_norm < self.error_tolerance_ori:
            ori_err = np.zeros_like(ori_err)
            w_err = np.zeros_like(w_err)

        ori_err *= -kp_kv[:, 0]
        w_err *= -kp_kv[:, 1]

        if dw_max > 0.0:
            ori_err_sq_norm = np.sum(ori_err**2)
            dw_max_sq = dw_max**2
            if ori_err_sq_norm > dw_max_sq:
                ori_err *= dw_max / np.sqrt(ori_err_sq_norm)

        return ori_err + w_err

    def compute_gains(self, gains, kd_values, method: str):
        if method == "dynamics":
            kp = np.asarray(gains)
            if kd_values is None:
                kd = self.damping_ratio * 2 * np.sqrt(kp)
            else:
                kd = np.asarray(kd_values)
        else:
            kp = np.asarray(gains)
            if kd_values is None:
                kd = self.damping_ratio * kp * self.integration_dt
            else:
                kd = np.asarray(kd_values)

        return np.stack([kp, kd], axis=-1)

    def control(self, pos: Optional[np.ndarray] = None, ori: Optional[np.ndarray] = None) -> np.ndarray:
        if pos is None:
            x_des = self.data.site_xpos[self.site_id]
        else:
            x_des = np.asarray(pos)
        if ori is None:
            xmat = self.data.site_xmat[self.site_id].reshape((3, 3))
            quat_des = tr.mat_to_quat(xmat)
        else:
            ori = np.asarray(ori)
            if ori.shape == (3, 3):
                quat_des = tr.mat_to_quat(ori)
            else:
                quat_des = ori

        kp_kv_pos = self.compute_gains(self.pos_gains, self.pos_kd, self.method)
        kp_kv_ori = self.compute_gains(self.ori_gains, self.ori_kd, self.method)

        ddx_max = self.max_pos_error if self.max_pos_error is not None else 0.0
        dw_max = self.max_ori_error if self.max_ori_error is not None else 0.0

        q = self.data.qpos[self.dof_ids]
        dq = self.data.qvel[self.dof_ids]

        J_v = np.zeros((3, self.model.nv), dtype=np.float64)
        J_w = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(self.model, self.data, J_v, J_w, self.site_id)
        J_v = J_v[:, self.dof_ids]
        J_w = J_w[:, self.dof_ids]
        J = np.concatenate([J_v, J_w], axis=0)

        x = self.data.site_xpos[self.site_id]
        dx = J_v @ dq
        ddx = self.pd_control(x, x_des, dx, kp_kv_pos, ddx_max)

        quat = tr.mat_to_quat(self.data.site_xmat[self.site_id].reshape((3, 3)))
        if quat @ quat_des < 0.0:
            quat *= -1.0
        w = J_w @ dq
        dw = self.pd_control_orientation(quat, quat_des, w, kp_kv_ori, dw_max)

        error = np.concatenate([ddx, dw], axis=0)

        if self.method == "dynamics":
            M = np.zeros((self.model.nv, self.model.nv), dtype=np.float64)
            mujoco.mj_fullM(self.model, M, self.data.qM)
            M = M[self.dof_ids, :][:, self.dof_ids]

            M_inv = np.linalg.inv(M)

            # Mx_inv = J @ M_inv @ J.T
            # if abs(np.linalg.det(Mx_inv)) >= 1e-2:
            #     Mx = np.linalg.inv(Mx_inv)
            # else:
            #     Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)
            ddq = M_inv @ J.T @ error
            dq += ddq * self.integration_dt
            # print("Error:", error)
            print("Acceleration (ddq):", ddq)
            # print("velocity:", dq)
            q += dq * self.integration_dt

        elif self.method == "pinv":
            J_pinv = np.linalg.pinv(J)
            dq = J_pinv @ error
            q += dq 
        elif self.method == "svd":
            U, S, Vt = np.linalg.svd(J, full_matrices=False)
            S_inv = np.zeros_like(S)
            for i in range(len(S)):
                if S[i] > 1e-5:
                    S_inv[i] = 1.0 / S[i]
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

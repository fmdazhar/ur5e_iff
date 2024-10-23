from typing import Optional, Tuple, Union
import time
import mujoco
import numpy as np
from dm_robotics.transformations import transformations as tr


def pd_control(
    x: np.ndarray,
    x_des: np.ndarray,
    dx: np.ndarray,
    kp_kv: np.ndarray,
    time_step,
    ddx_max: float = 0.0,
    error_tolerance: float = 0.01,
) -> np.ndarray:
    # Compute error.
    x_err = x - x_des
    dx_err = dx

    # Compute the norm of the error.
    x_err_norm = np.linalg.norm(x_err)

    # Apply error tolerance.
    if x_err_norm < error_tolerance:
        x_err = np.zeros_like(x_err)
        dx_err = np.zeros_like(dx_err)

    # Apply gains.
    x_err *= -kp_kv[:, 0] / time_step
    dx_err *= -kp_kv[:, 1]

    # Limit maximum error.
    if ddx_max > 0.0:
        x_err_sq_norm = np.sum(x_err**2)
        ddx_max_sq = ddx_max**2
        if x_err_sq_norm > ddx_max_sq:
            x_err *= ddx_max / np.sqrt(x_err_sq_norm)

    return x_err + dx_err


def pd_control_orientation(
    quat: np.ndarray,
    quat_des: np.ndarray,
    w: np.ndarray,
    kp_kv: np.ndarray,
    time_step,
    dw_max: float = 0.0,
    error_tolerance: float = 0.01,
) -> np.ndarray:
    # Compute error.
    quat_err = tr.quat_diff_active(source_quat=quat_des, target_quat=quat)
    ori_err = tr.quat_to_axisangle(quat_err)
    w_err = w


    # Compute the norm of the orientation error.
    ori_err_norm = np.linalg.norm(ori_err)

    # Apply error tolerance.
    if ori_err_norm < error_tolerance:
        ori_err = np.zeros_like(ori_err)
        w_err = np.zeros_like(w_err)

    # Apply gains.
    ori_err *= -kp_kv[:, 0] / time_step
    w_err *= -kp_kv[:, 1] 

    # Limit maximum error.
    if dw_max > 0.0:
        ori_err_sq_norm = np.sum(ori_err**2)
        dw_max_sq = dw_max**2
        if ori_err_sq_norm > dw_max_sq:
            ori_err *= dw_max / np.sqrt(ori_err_sq_norm)

    return ori_err + w_err

def cartesain_motion_controller(
    model,
    data,
    site_id,
    integration_dt,
    dof_ids: np.ndarray,
    pos: Optional[np.ndarray] = None,
    ori: Optional[np.ndarray] = None,
    # pos_gains: Union[Tuple[float, float, float], np.ndarray] = (100000.0, 100000.0, 100000.0),
    # ori_gains: Union[Tuple[float, float, float], np.ndarray] = (50000.0, 50000.0, 50000.0),
    pos_gains: Union[Tuple[float, float, float], np.ndarray] = (1, 1, 1),
    ori_gains: Union[Tuple[float, float, float], np.ndarray] = (1, 1, 1),
    damping_ratio: float = 1.0,
    max_pos_acceleration: Optional[float] = None,
    max_ori_acceleration: Optional[float] = None,
    max_angvel: Optional[float] = 0.0,
    error_tolerance_pos: float = 0.01,
    error_tolerance_ori: float = 0.01,
) -> np.ndarray:
    if pos is None:
        x_des = data.site_xpos[site_id]
    else:
        x_des = np.asarray(pos)
    if ori is None:
        xmat = data.site_xmat[site_id].reshape((3, 3))
        quat_des = tr.mat_to_quat(xmat.reshape((3, 3)))
    else:
        ori = np.asarray(ori)
        if ori.shape == (3, 3):
            quat_des = tr.mat_to_quat(ori)
        else:
            quat_des = ori


    kp = np.asarray(pos_gains)
    kd = damping_ratio * 2 * np.sqrt(kp)
    kp_kv_pos = np.stack([kp, kd], axis=-1)

    kp = np.asarray(ori_gains)
    kd = damping_ratio * 2 * np.sqrt(kp)
    kp_kv_ori = np.stack([kp, kd], axis=-1)

    ddx_max = max_pos_acceleration if max_pos_acceleration is not None else 0.0
    dw_max = max_ori_acceleration if max_ori_acceleration is not None else 0.0


    # Get current state.
    q = data.qpos[dof_ids]
    dq = data.qvel[dof_ids]

    # Compute Jacobian of the eef site in world frame.
    J_v = np.zeros((3, model.nv), dtype=np.float64)
    J_w = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(
        model,
        data,
        J_v,
        J_w,
        site_id,
    )
    J_v = J_v[:, dof_ids]
    J_w = J_w[:, dof_ids]
    J = np.concatenate([J_v, J_w], axis=0)

    # Compute position PD control.
    x = data.site_xpos[site_id]
    dx = J_v @ dq
    ddx = pd_control(
        x=x,
        x_des=x_des,
        dx=dx,
        kp_kv=kp_kv_pos,
        ddx_max=ddx_max,
        error_tolerance=error_tolerance_pos,
        time_step= integration_dt,
    )

    # Compute orientation PD control.
    quat = tr.mat_to_quat(data.site_xmat[site_id].reshape((3, 3)))
    if quat @ quat_des < 0.0:
        quat *= -1.0
    w = J_w @ dq
    dw = pd_control_orientation(
        quat=quat,
        quat_des=quat_des,
        w=w,
        kp_kv=kp_kv_ori,
        dw_max=dw_max,
        error_tolerance=error_tolerance_ori,
        time_step= integration_dt,
    )

    # Compute generalized forces.
    ddx_dw = np.concatenate([ddx, dw], axis=0)

    # Compute inertia matrix in joint space.
    M = np.zeros((model.nv, model.nv), dtype=np.float64)
    mujoco.mj_fullM(model, M, data.qM)
    M = M[dof_ids, :][:, dof_ids]

    # Compute inertia matrix in task space.
    M_inv = np.linalg.inv(M)
    Mx_inv = J @ M_inv @ J.T
    if abs(np.linalg.det(Mx_inv)) >= 1e-2:
        Mx = np.linalg.inv(Mx_inv)
    else:
        Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)


    # # Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
    # ddq = M_inv @ J.T @ ddx_dw

    # # Update the joint velocities.
    # dq += ddq * integration_dt

    # # Update the joint positions.
    # q += dq * integration_dt

    # Solve system of equations: J @ dq = error.
    # Damping term for the pseudoinverse. This is used to prevent joint velocities from
    # becoming too large when the Jacobian is close to singular.
    damping: float = 1e-4
    diag = damping * np.eye(6)
    dq = J.T @ np.linalg.solve(J @ J.T + diag, ddx_dw)

    # Update the joint positions.
    q += dq * integration_dt

    # Clip the control signal to be within the joint limits.
    q_min = model.jnt_range[:6, 0]  # Minimum joint limits for the first 6 joints
    q_max = model.jnt_range[:6, 1]  # Maximum joint limits for the first 6 joints
    np.clip(q, q_min, q_max, out=q)

    return q
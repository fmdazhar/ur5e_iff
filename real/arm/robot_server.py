
"""
A general robot arm
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python import trac_ik
import rospy
import numpy as np
import utils.common as arutil
import copy
from utils.ros_util import joints_to_kdl
from utils.ros_util import kdl_array_to_numpy
from utils.ros_util import kdl_frame_to_numpy
from utils.ai_logger import Logger


class RobotServer:
    def __init__(self, cfgs):
        self.cfgs = cfgs
        self._init_real_consts()


    def _init_real_consts(self):
        """
        Initialize constants.
        """
        self._home_pose = self.cfgs.ARM.HOME_POSE
        self._reset_position = self.cfgs.ARM.RESET_POSITION
        robot_description = self.cfgs.ROBOT_DESCRIPTION
        urdf_string = rospy.get_param(robot_description)
        self._num_ik_solver = trac_ik.IK(self.cfgs.ARM.ROBOT_BASE_FRAME,
                                         self.cfgs.ARM.ROBOT_EE_FRAME,
                                         urdf_string=urdf_string)
        _, urdf_tree = treeFromParam(robot_description)
        base_frame = self.cfgs.ARM.ROBOT_BASE_FRAME
        ee_frame = self.cfgs.ARM.ROBOT_EE_FRAME
        self._urdf_chain = urdf_tree.getChain(base_frame,
                                              ee_frame)
        self.arm_jnt_names = self._get_kdl_joint_names()
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_jnt_names)
        self._jac_solver = kdl.ChainJntToJacSolver(self._urdf_chain)
        self._fk_solver_pos = kdl.ChainFkSolverPos_recursive(self._urdf_chain)
        self._fk_solver_vel = kdl.ChainFkSolverVel_recursive(self._urdf_chain)


    def is_jpos_in_good_range(self):
        """Check if joint positions are within acceptable range."""
        raise NotImplementedError

    def get_jpos(self, joint_name=None):
        """Get current joint positions."""
        raise NotImplementedError

    def get_jvel(self, joint_name=None):
        """Get current joint velocities."""
        raise NotImplementedError

    def get_ee_pose(self):
        """Get current end-effector pose."""
        raise NotImplementedError

    def get_ee_vel(self):
        """Get current end-effector velocity."""
        raise NotImplementedError

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """Set joint positions."""
        raise NotImplementedError

    def set_jvel(self, velocity, wait=False, *args, **kwargs):
        """Set joint velocities."""
        raise NotImplementedError

    def get_ee_pose(self):
        """
        Get current cartesian pose of the EE, in the robot's base frame,
        using ROS subscriber to the tf tree topic.

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3]`).
            - np.ndarray: quaternion representation ([x, y, z, w]) of the EE
              orientation (shape: :math:`[4]`).
            - np.ndarray: rotation matrix representation of the EE orientation
              (shape: :math:`[3, 3]`).
            - np.ndarray: euler angle representation of the EE orientation
              (roll, pitch, yaw with static reference frame)
              (shape: :math:`[3]`).
        """
        raise NotImplementedError
    
    def move_ee_xyz(self, delta_xyz, wait=True, *args, **kwargs):
        """Move end-effector in XYZ directions."""
        raise NotImplementedError

    def compute_ik(self, pos, ori=None, qinit=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector
        (self.cfgs.ARM.ROBOT_EE_FRAME).

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`).
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation.
            qinit (list or np.ndarray): initial joint positions for numerical
                IK (shape: :math:`[6,]`).

        Returns:
            list: inverse kinematics solution (joint angles)
        """
        if ori is not None:
            ee_quat = arutil.to_quat(ori)
        else:
            ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()
        ori_x = ee_quat[0]
        ori_y = ee_quat[1]
        ori_z = ee_quat[2]
        ori_w = ee_quat[3]
        if qinit is None:
            qinit = self.get_jpos().tolist()
        elif isinstance(qinit, np.ndarray):
            qinit = qinit.flatten().tolist()
        pos_tol = self.cfgs.ARM.IK_POSITION_TOLERANCE
        ori_tol = self.cfgs.ARM.IK_ORIENTATION_TOLERANCE
        jnt_poss = self._num_ik_solver.get_ik(qinit,
                                              pos[0],
                                              pos[1],
                                              pos[2],
                                              ori_x,
                                              ori_y,
                                              ori_z,
                                              ori_w,
                                              pos_tol,
                                              pos_tol,
                                              pos_tol,
                                              ori_tol,
                                              ori_tol,
                                              ori_tol)
        if jnt_poss is None:
            return None
        return list(jnt_poss)

    def compute_fk_position(self, jpos, tgt_frame):
        """
        Given joint angles, compute the pose of desired_frame with respect
        to the base frame (self.cfgs.ARM.ROBOT_BASE_FRAME). The desired frame
        must be in self.arm_link_names.

        Args:
            jpos (list or flattened np.ndarray): joint angles.
            tgt_frame (str): target link frame.

        Returns:
            2-element tuple containing

            - np.ndarray: translational vector (shape: :math:`[3,]`).
            - np.ndarray: rotational matrix (shape: :math:`[3, 3]`).
        """
        if isinstance(jpos, list):
            jpos = np.array(jpos)
        jpos = jpos.flatten()
        if jpos.size != self.arm_dof:
            raise ValueError('Length of the joint angles '
                             'does not match the robot DOF')
        assert jpos.size == self.arm_dof
        kdl_jnt_angles = joints_to_kdl(jpos)

        kdl_end_frame = kdl.Frame()
        idx = self.arm_link_names.index(tgt_frame) + 1
        fg = self._fk_solver_pos.JntToCart(kdl_jnt_angles,
                                           kdl_end_frame,
                                           idx)
        if fg < 0:
            raise ValueError('KDL Pos JntToCart error!')
        pose = kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].flatten()
        rot = pose[:3, :3]
        return pos, rot

    def compute_fk_velocity(self, jpos, jvel, tgt_frame):
        """
        Given joint_positions and joint velocities,
        compute the velocities of tgt_frame with respect
        to the base frame.

        Args:
            jpos (list or flattened np.ndarray): joint positions.
            jvel (list or flattened np.ndarray): joint velocities.
            tgt_frame (str): target link frame.

        Returns:
            np.ndarray: translational velocity and rotational velocity
            (vx, vy, vz, wx, wy, wz) (shape: :math:`[6,]`).
        """
        if isinstance(jpos, list):
            jpos = np.array(jpos)
        if isinstance(jvel, list):
            jvel = np.array(jvel)
        kdl_end_frame = kdl.FrameVel()
        kdl_jnt_angles = joints_to_kdl(jpos)
        kdl_jnt_vels = joints_to_kdl(jvel)
        kdl_jnt_qvels = kdl.JntArrayVel(kdl_jnt_angles, kdl_jnt_vels)
        idx = self.arm_link_names.index(tgt_frame) + 1
        fg = self._fk_solver_vel.JntToCart(kdl_jnt_qvels,
                                           kdl_end_frame,
                                           idx)
        if fg < 0:
            raise ValueError('KDL Vel JntToCart error!')
        end_twist = kdl_end_frame.GetTwist()
        return np.array([end_twist[0], end_twist[1], end_twist[2],
                         end_twist[3], end_twist[4], end_twist[5]])
    
    def _get_kdl_link_names(self):
        """
        Internal method to get the link names from the KDL URDF chain.

        Returns:
            list: List of link names.
        """
        num_links = self._urdf_chain.getNrOfSegments()
        link_names = []
        for i in range(num_links):
            link_names.append(self._urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def get_jacobian(self, joint_angles):
        """
        Return the geometric jacobian on the given joint angles.
        Refer to P112 in "Robotics: Modeling, Planning, and Control".

        Args:
            joint_angles (list or flattened np.ndarray): joint angles.

        Returns:
            np.ndarray: jacobian (shape: :math:`[6, DOF]`).
        """
        q = kdl.JntArray(self._urdf_chain.getNrOfJoints())
        for i in range(q.rows()):
            q[i] = joint_angles[i]
        jac = kdl.Jacobian(self._urdf_chain.getNrOfJoints())
        fg = self._jac_solver.JntToJac(q, jac)
        if fg < 0:
            raise ValueError('KDL JntToJac error!')
        jac_np = kdl_array_to_numpy(jac)
        return jac_np
    
    def _get_kdl_joint_names(self):
        """
        Internal method to get the joint names from the KDL URDF chain.

        Returns:
            list: List of joint names.
        """
        num_links = self._urdf_chain.getNrOfSegments()
        num_joints = self._urdf_chain.getNrOfJoints()
        joint_names = []
        for i in range(num_links):
            link = self._urdf_chain.getSegment(i)
            joint = link.getJoint()
            joint_type = joint.getType()
            # JointType definition: [RotAxis,RotX,RotY,RotZ,TransAxis,
            #                        TransX,TransY,TransZ,None]
            if joint_type > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)


    def _get_tip_transform(self):
        """
        Internal method to get the transform between the robot's
        wrist and the tip of the gripper.

        Returns:
            2-element tuple containing

            - list: Translation component of the gripper tip transform
              (shape :math:`[3,]`).
            - list: Euler angle orientation component of the gripper
              tip transform. (shape :math:`[3,]`).
        """
        ee_frame = self.cfgs.ARM.ROBOT_EE_FRAME
        gripper_tip_id = self.arm_link_names.index(ee_frame)
        gripper_tip_link = self._urdf_chain.getSegment(gripper_tip_id)
        gripper_tip_tf = kdl_frame_to_numpy(gripper_tip_link.getFrameToTip())
        gripper_tip_pos = gripper_tip_tf[:3, 3].flatten()
        gripper_tip_rot_mat = gripper_tip_tf[:3, :3]
        gripper_tip_euler = arutil.rot2euler(gripper_tip_rot_mat)
        return list(gripper_tip_pos), list(gripper_tip_euler)

    def _set_tool_offset(self):
        """
        Internal method to send a URScript command to the robot so that
        it updates it tool center point variable to match the URDF.
        """
        tool_offset_prog = 'set_tcp(p[%f, %f, %f, %f, %f, %f])' % (
            self.gripper_tip_pos[0],
            self.gripper_tip_pos[1],
            self.gripper_tip_pos[2],
            self.gripper_tip_ori[0],
            self.gripper_tip_ori[1],
            self.gripper_tip_ori[2]
        )

        self._output_pendant_msg(tool_offset_prog)
        self._send_urscript(tool_offset_prog)
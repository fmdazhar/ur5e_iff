from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numbers
import time
import sys
import threading


import numpy as np
import rospy
import tf
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python import trac_ik

import utils.common as arutil
from utils.arm_util import wait_to_reach_jnt_goal, wait_to_reach_ee_goal
from geometry_msgs.msg import PoseStamped

class UR5eReal:
    """
    A Class for interfacing with a real UR5e robot.
    Combines functionalities from the ARM, SingleArmReal, and SingleArmROS classes.

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        moveit_planner (str): motion planning algorithm.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.
        wrist_cam (bool): whether the robot has a wrist camera mounted.
    """
    def __init__(self, cfgs):
        self.cfgs = cfgs
        self._init_real_consts()
        self._init_ros_consts()
        self._setup_pub_sub()
        self.is_jpos_in_good_range()


    def _init_real_consts(self):
        """
        Initialize constants.
        """
        self._home_position = self.cfgs.ARM.HOME_POSITION

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

        self.ee_link = self.cfgs.ARM.ROBOT_EE_FRAME

    def _init_ros_consts(self):
        """
        Initialize constants
        """
        # read the joint limit (max velocity and acceleration) from the
        # moveit configuration file
        jnt_params = []
        max_vels = []
        max_accs = []
        for arm_jnt in self.arm_jnt_names:
            jnt_param = self.cfgs.ROBOT_DESCRIPTION + \
                        '_planning/joint_limits/' + arm_jnt
            jnt_params.append(copy.deepcopy(jnt_param))
            max_vels.append(rospy.get_param(jnt_param + '/max_velocity'))
            max_accs.append(rospy.get_param(jnt_param + '/max_acceleration'))
        self.max_vel = np.min(max_vels)
        self.max_acc = np.min(max_accs)

        self._j_pos = dict()
        self._j_vel = dict()
        self._j_torq = dict()
        self._j_state_lock = threading.RLock()
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(self.cfgs.ARM.ROSTOPIC_JOINT_STATES, JointState,
                         self._callback_joint_states)
        

    def _setup_pub_sub(self):
        """
        Initialize all the publishers and subscribers used internally.
        """
        # for publishing end effector pose to real robot
        self._ee_pose_pub = rospy.Publisher(
            self.cfgs.ARM.END_EFFECTOR_POSE_TOPIC,
            PoseStamped,
            queue_size=3
        )

        time.sleep(1)
    
    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for arm joint states

        Args:
            msg (sensor_msgs/JointState): Contains message published in topic.
        """
        self._j_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.arm_jnt_names:
                if idx < len(msg.position):
                    self._j_pos[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._j_vel[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._j_torq[name] = msg.effort[idx]
        self._j_state_lock.release()


    def is_jpos_in_good_range(self):
        """
        Check if the joint angles lie in (-pi, pi].

        Returns:
            bool: whether the joint angles are in (-pi, pi].

        """
        jposs = self.get_jpos()
        for i, jpos in enumerate(jposs):
            if jpos <= -np.pi or jpos > np.pi:
                Logger.log_warn('Current joint angles are: %s\n'
                            'Some joint angles are outside of the valid'
                            ' range (-pi, pi]\n Please use the Teaching'
                            ' Pendant to move the correponding joints so'
                            ' that all the joint angles are within (-pi,'
                            ' pi]!' % str(jposs))
                return False
        return True
    
    def get_jpos(self, joint_name=None):
        """
        Gets the current joint position of the robot. Gets the value
        from the internally updated dictionary that subscribes to the ROS
        topic /joint_states.

        Args:
            joint_name (str, optional): If it's None,
                it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint.

        Returns:
            One of the following

            - float: joint position given joint_name.
            - list: joint positions if joint_name is None
              (shape: :math:`[DOF]`).

        """
        self._j_state_lock.acquire()
        if joint_name is not None:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] '
                                'not recognized!' % joint_name)
            jpos = self._j_pos[joint_name]
        else:
            jpos = []
            for joint in self.arm_jnt_names:
                jpos.append(self._j_pos[joint])
        self._j_state_lock.release()
        return jpos

    def get_jvel(self, joint_name=None):
        """
        Gets the current joint angular velocities of the robot. Gets the value
        from the internally updated dictionary that subscribes to the ROS
        topic /joint_states.

        Args:
            joint_name (str, optional): If it's None,
                it will return joint velocities
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint.

        Returns:
            One of the following

            - float: joint velocity given joint_name.
            - list: joint velocities if joint_name is None
              (shape: :math:`[DOF]`).
        """
        self._j_state_lock.acquire()
        if joint_name is not None:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] not recognized!' % joint_name)
            jvel = self._j_vel[joint_name]
        else:
            jvel = []
            for joint in self.arm_jnt_names:
                jvel.append(self._j_vel[joint])
        self._j_state_lock.release()
        return jvel
    

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
        pos, quat = get_tf_transform(self.tf_listener,
                                     self.cfgs.ARM.ROBOT_BASE_FRAME,
                                     self.cfgs.ARM.ROBOT_EE_FRAME)
        rot_mat = arutil.quat2rot(quat)
        euler_ori = arutil.quat2euler(quat)
        return np.array(pos), np.array(quat), rot_mat, euler_ori
    
    def get_ee_vel(self):
        """
        Return the end effector's velocity.

        Returns:
            2-element tuple containing

            - np.ndarray: translational velocity (vx, vy, vz)
              (shape: :math:`[3,]`).
            - np.ndarray: rotational velocity
              (wx, wy, wz) (shape: :math:`[3,]`).
        """
        jpos = self.get_jpos()
        jvel = self.get_jvel()
        ee_vel = self.compute_fk_velocity(jpos, jvel,
                                          self.cfgs.ARM.ROBOT_EE_FRAME)
        return ee_vel[:3], ee_vel[3:]
    

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad).

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
                (shape: :math:`[6,]` if list, otherwise a single value).
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None.
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        if position is None:
            return False  # No positions specified

        success = False

        if not rospy.is_shutdown():
            joint_goal = JointState()
            joint_goal.header.stamp = rospy.Time.now()
            joint_goal.name = self.cfgs.ARM.JOINT_NAMES
            joint_goal.position = position

            try:
                self._joint_pos_pub.publish(joint_goal)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                pass

        if wait:
            success = wait_to_reach_jnt_goal(
                position,
                get_func=self.get_jpos,
                joint_name=joint_name,
                get_func_derv=self.get_jvel,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_ERROR
            )

        return success


    def set_jvel(self, velocity, wait=False,
                 *args, **kwargs):
        """
        Set joint velocity command to the robot (units in rad/s).

        Args:
            velocity (float or list or flattened np.ndarray): list of target
                joint velocity value(s)
                (shape: :math:`[6,]` if list, otherwise a single value)
            joint_name (str, optional): If not provided, velocity should be
                list and all joints will be turned on at specified velocity.
                Defaults to None.
            wait (bool, optional): If True, block until robot reaches
                desired joint velocity value(s). Defaults to False.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        velocity = copy.deepcopy(velocity)
        success = False

        if velocity is None:
            return False  # No velocities specified

        if not rospy.is_shutdown():
            velocity_command = JointState()
            velocity_command.header.stamp = rospy.Time.now()
            velocity_command.name = self.cfgs.ARM.JOINT_NAMES
            velocity_command.velocity = joint_velocities

            try:
                self._joint_vel_pub.publish(velocity_command)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                pass

        if wait:
            success = wait_to_reach_jnt_goal(
                velocity,
                get_func=self.get_jvel,
                joint_name=joint_name,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_VEL_ERROR
            )

        return success
    

    def set_ee_pose(self, pos=None, ori=None, wait=True, *args, **kwargs):
        """
        Set cartesian space pose of end effector.

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`).
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation.
            wait (bool): wait until the motion completes.
            ik_first (bool, optional): Whether to use the solution computed
                by IK, or to use UR built in movel function which moves
                linearly in tool space (movel may sometimes fail due to
                sinularities). This parameter takes effect only when
                self._use_urscript is True. Defaults to False.

        Returns:
            bool: Returns True is robot successfully moves to goal pose.
        """
        if ori is None and pos is None:
            return True
        success = False
        if ori is None:
            pose = self.get_ee_pose()  # last index is euler angles
            quat = pose[1]
        else:
            quat = arutil.to_quat(ori)
        if pos is None:
            pose = self.get_ee_pose()
            pos = pose[0]

        if not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.cfgs.ARM.ROBOT_BASE_FRAME
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            try:
                self._ee_pose_pub.publish(pose)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                # This rarely happens on killing this node.
                pass
        
        if wait:
            args_dict = {
                'get_func': self.get_ee_pose,
                'get_func_derv': self.get_ee_vel,
                'timeout': self.cfgs.ARM.TIMEOUT_LIMIT,
                'pos_tol': self.cfgs.ARM.MAX_EE_POS_ERROR,
                'ori_tol': self.cfgs.ARM.MAX_EE_ORI_ERROR
            }
            success = wait_to_reach_ee_goal(pos, quat,
                                            **args_dict)
        return success
    
    def get_ee_vel(self):
        """
        Return the end effector's velocity.

        Returns:
            2-element tuple containing

            - np.ndarray: translational velocity (vx, vy, vz)
              (shape: :math:`[3,]`).
            - np.ndarray: rotational velocity
              (wx, wy, wz) (shape: :math:`[3,]`).
        """
        jpos = self.get_jpos()
        jvel = self.get_jvel()
        ee_vel = self.compute_fk_velocity(jpos, jvel,
                                          self.cfgs.ARM.ROBOT_EE_FRAME)
        return ee_vel[:3], ee_vel[3:]
    

    def move_ee_xyz(self, delta_xyz, wait=True,
                    *args, **kwargs):
        """
        Move end effector in straight line while maintaining orientation.

        Args:
            delta_xyz (list or np.ndarray): Goal change in x, y, z position of
                end effector.
            eef_step (float, optional): Discretization step in cartesian space
                for computing waypoints along the path. Defaults to 0.005 (m).
            wait (bool, optional): True if robot should not do anything else
                until this goal is reached, or the robot times out.
                Defaults to True.

        Returns:
            bool: True if robot successfully reached the goal pose.
        """
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        ee_pos[0] += delta_xyz[0]
        ee_pos[1] += delta_xyz[1]
        ee_pos[2] += delta_xyz[2]
        success = self.set_ee_pose(ee_pos, ee_euler, wait=wait)
        return success


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
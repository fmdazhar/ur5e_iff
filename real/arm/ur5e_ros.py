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

from arm.robot_server import RobotServer
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped


import utils.common as arutil
from utils.controller_utils import ControllerUtil
from utils.ai_logger import Logger
from utils.ros_util import get_tf_transform
from utils.arm_util import wait_to_reach_jnt_goal, wait_to_reach_ee_goal

class UR5eRealServer(RobotServer):
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
    def __init__(self, cfgs, reset_joint_target=None):
        # Call the parent class's constructor to initialize shared attributes
        super(UR5eRealServer, self).__init__(cfgs)
        self._init_ros_consts()
        self._setup_pub_sub()
        self.is_jpos_in_good_range()
        self.controller_util = ControllerUtil()  # Instantiate ControllerUtil here

        # Overwrite _reset_position if reset_joint_target is provided
        if reset_joint_target is not None:
            self._reset_position = reset_joint_target
        # Wait until controller spawner is done
        time.sleep(1)

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
        
        self._wrench_lock = threading.RLock()
        self._wrench = {'force': [0, 0, 0], 'torque': [0, 0, 0]}

        # Subscriber for force-torque sensor wrench topic
        rospy.Subscriber(self.cfgs.ARM.ROSTOPIC_WRENCH, WrenchStamped,
                         self._callback_wrench)
        
    def _setup_pub_sub(self):
        """
        Initialize all the publishers and subscribers used internally.
        """
        # for publishing end effector pose to real robot
        self._ee_pose_pub = rospy.Publisher(
            self.cfgs.ARM.ROBOT_EE_POSE_COMMAND_TOPIC,
            PoseStamped,
            queue_size=3
        )
        self._joint_pos_pub = rospy.Publisher(
            self.cfgs.ARM.ROBOT_JOINT_COMMAND_TOPIC,
            JointTrajectory,
            queue_size=10
        )
        self._urscript_pub = rospy.Publisher(
            self.cfgs.ARM.URSCRIPT_TOPIC,
            String,
            queue_size=10
        )
        time.sleep(1)

    def _callback_wrench(self, msg):
        """
        ROS subscriber callback for force-torque sensor wrench data

        Args:
            msg (geometry_msgs/WrenchStamped): Contains wrench data published in topic.
        """
        self._wrench_lock.acquire()
        try:
            self._wrench['force'][0] = msg.wrench.force.x
            self._wrench['force'][1] = msg.wrench.force.y
            self._wrench['force'][2] = msg.wrench.force.z
            self._wrench['torque'][0] = msg.wrench.torque.x
            self._wrench['torque'][1] = msg.wrench.torque.y
            self._wrench['torque'][2] = msg.wrench.torque.z
        finally:
            self._wrench_lock.release()

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
                Logger.warning('Current joint angles are: %s\n'
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
    
    def get_wrench(self):
        """
        Gets the current wrench (force and torque) of the force-torque sensor.

        Returns:
            list: Contains force and torque as a flattened list with x, y, z components.
                [force_x, force_y, force_z, torque_x, torque_y, torque_z]
        """
        self._wrench_lock.acquire()
        try:
            # Combine force and torque into a single list
            wrench_list = self._wrench['force'][:] + self._wrench['torque'][:]
        finally:
            self._wrench_lock.release()

        return wrench_list

    
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
    
    def set_jpos(self, position, joint_name=None, wait=True, _use_urscript=False, *args, **kwargs):
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
        position = copy.deepcopy(position)
        success = False

        if joint_name is None:
            if len(position) != self.arm_dof:
                raise ValueError('position should contain %d elements if '
                                 'joint_name is not provided' % self.arm_dof)
            tgt_pos = position
        else:
            if not isinstance(position, numbers.Number):
                raise TypeError('position should be individual float value'
                                ' if joint_name is provided')
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = self.get_jpos()
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_pos[arm_jnt_idx] = position

        if _use_urscript:
            prog = 'movej([%f, %f, %f, %f, %f, %f],' \
                   ' a=%f, v=%f)' % (tgt_pos[0],
                                     tgt_pos[1],
                                     tgt_pos[2],
                                     tgt_pos[3],
                                     tgt_pos[4],
                                     tgt_pos[5],
                                     self._motion_acc,
                                     self._motion_vel)
            self._send_urscript(prog)

        else:
            try:
                self._pub_joint_pos(position)
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

    def _pub_joint_pos(self, position):
        """
        Received joint velocity command, puts it in a ROS trajectory
        msg, and uses internal publisher to send to /joint_speed topic
        on the real robot.

        Args:
            velocity (list): List of desired joint velocities
                (length and order have already been checked).
        """
        goal_pos_msg = JointTrajectory()
        goal_pos_msg.points.append(
            JointTrajectoryPoint(
                positions=position))
        self._joint_pos_pub.publish(goal_pos_msg)

    def reset_joint(self, position, use_urscript=False):
        """Resets Joints (needed after running for hours)"""

        if position is None:
            position = self._reset_position

        success = False
        if use_urscript:
            # Use URScript
            success = self.set_jpos(position, use_urscript = True, wait=True)
        else:
            # First motion controller
            try:
                self.controller_util.switch_on_controller(self.cfgs.ARM.ROBOT_JOINT_TRAJECTORY_CONTROLLER,
                                        exclude_controllers=[self.cfgs.ARM.ROBOT_CARTESIAN_MOTION_CONTROLLER, self.cfgs.EETOOL.GRIPPER_ACTION_CONTROLLER])
            except Exception as e:
                print(f"Error during joint reset: {e}")
            time.sleep(3)

            # Launch joint controller reset
            print("RUNNING JOINT RESET")
            # Wait until target joint angles are reached
            success = self.set_jpos(position, wait=True)
            # Stop joint controller
            if success:
                print("RESET DONE")
            else:
                current_positions = self.get_jpos()  # Get current joint positions
                print("JOINT RESET TIMEOUT", current_positions)

            #start impedance back
            try:
                self.controller_util.switch_on_controller(self.cfgs.ARM.ROBOT_CARTESIAN_MOTION_CONTROLLER,
                                        exclude_controllers=[self.cfgs.ARM.ROBOT_CARTESIAN_MOTION_CONTROLLER, self.cfgs.EETOOL.GRIPPER_ACTION_CONTROLLER])
            except Exception as e:
                print(f"Error during controller switch: {e}")
            time.sleep(1)
            print("KILLED JOINT RESET")

        return success

    def go_to_rest(self, pose, joint_reset = False, random_reset = False, timeout = 1.5):
        """
        Move the robot to a pre-defined home pose.
        """
        # controller_name = "pos_joint_traj_controller"
        # if self.check_state(controller_name, "active"):
        #     self.set_jpos(self._home_position, wait=True)
        #     print("[INFO] Robot is moving to the home position.")
        # else:
        #     print(f"[ERROR] Controller '{controller_name}' is not active. Cannot move to home position.")
        if pose is None:
            pose = self._home_pose
        success = False
        pos = pose[:3]
        ori = pose[3:]
        
        # Perform joint reset if needed
        if joint_reset:
            print("JOINT RESET")
            # use movej from URScript
            self.reset_joint(self, use_urscript=False)

        # Initialize reset_pos and reset_ori
        reset_pos = pos.copy()
        reset_ori = ori.copy()

        # Perform Carteasian reset
        if random_reset:  # randomize reset position in xy plane
            reset_pos[:2] += np.random.uniform(
                -self.cfgs.ARM.random_xy_range, self.cfgs.ARM.random_xy_range, (2,)
            )
            euler_random = arutil.to_euler_angles(_)
            euler_random[-1] += np.random.uniform(
                -self.cfgs.ARM.random_rz_range, self.cfgs.ARM.random_rz_range
            )
            reset_ori[3:] = arutil.euler_2_quat(euler_random)


        # Direct movement to the target position without interpolation
        success = self.set_ee_pose(reset_pos, reset_ori)

        if not success:
                print('Robot go_home failed!!!')

    def set_ee_pose(self, pos=None, ori=None, wait=True, clip=False, interpolate=False, 
                timeout=None,
                pos_tol=None,
                ori_tol=None, *args, **kwargs):
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

        # Assign default values if None
        if timeout is None:
            timeout = self.cfgs.ARM.TIMEOUT_LIMIT
        if pos_tol is None:
            pos_tol = self.cfgs.ARM.MAX_EE_POS_ERROR
        if ori_tol is None:
            ori_tol = self.cfgs.ARM.MAX_EE_ORI_ERROR

        if ori is None and pos is None:
            return True

        success = False
        if ori is None:
            pose = self.get_ee_pose()  # last index is euler angles
            quat = pose[1]
        elif clip:
                euler = arutil.to_euler_angles(ori)
                # Clip first euler angle separately due to discontinuity from pi to -pi
                sign = np.sign(euler[0])
                euler[0] = sign * (
                    np.clip(
                        np.abs(euler[0]),
                        self.cfgs.ARM.rpy_bounding_box_low[0],
                        self.cfgs.ARM.rpy_bounding_box_high[0],
                    )
                )
                euler[1:] = np.clip(
                    euler[1:], self.cfgs.ARM.rpy_bounding_box_low[1:], self.cfgs.ARM.rpy_bounding_box_high[1:]
                )
                quat = arutil.to_quat(euler)
        else:
            quat = arutil.to_quat(ori)

        if pos is None:
            pose = self.get_ee_pose()
            pos = pose[0]
        elif clip:
            pos = np.clip(
            pos, self.cfgs.ARM.xyz_bounding_box_low, self.cfgs.ARM.xyz_bounding_box_high
        )
            
        if interpolate:
            # Interpolate over a specified timeout
            steps = int(timeout * self.cfgs.ARM.CONTROL_FREQUENCY)
            ee_pos = self.get_ee_pose()

            # Create a linear path for positions
            path = np.linspace(ee_pos, pos, steps)

            for p in path:
                # Create a PoseStamped message for each step
                self._publish_pose(p, quat)
                time.sleep(1 / self.cfgs.ARM.CONTROL_FREQUENCY)
            
        else:
            # Direct movement to the target position without interpolation
            self._publish_pose(pos, quat)

        if wait:
            args_dict = {
                'get_func': self.get_ee_pose,
                'get_func_derv': self.get_ee_vel,
                'timeout': timeout,
                'pos_tol': pos_tol,
                'ori_tol': ori_tol
            }
            success = wait_to_reach_ee_goal(pos, quat,
                                            **args_dict)
        return success
    
    def _publish_pose(self, pos, quat):
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
                pass

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


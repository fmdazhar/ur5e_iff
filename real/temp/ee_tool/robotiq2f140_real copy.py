import rospy
import threading
import time
import numpy as np

from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as OutputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as InputMsg

from airobot.ee_tool.ee import EndEffectorTool
from airobot.utils.common import clamp
from airobot.utils.common import print_red

class RobotiqCGripperReal(EndEffectorTool):
    """
    Class for interfacing with the Robotiq C-Gripper when
    it is attached to a robot arm. Communication with the gripper
    is through ROS topics.

    Args:
        cfgs (YACS CfgNode): Configurations for the gripper.

    Attributes:
        cfgs (YACS CfgNode): Configurations for the gripper.
        jnt_names (list): List of joint names of the gripper.
    """

    def __init__(self, cfgs):
        super(RobotiqCGripperReal, self).__init__(cfgs=cfgs)
        self.jnt_names = ['finger_joint']
        self.cur_status = None
        self._get_state_lock = threading.RLock()
        self._pub_state_lock = threading.RLock()

        # Publishers and Subscribers
        self._cmd_pub = rospy.Publisher(
            'Robotiq2FGripperRobotOutput', OutputMsg, queue_size=10)
        self._status_sub = rospy.Subscriber(
            'Robotiq2FGripperRobotInput', InputMsg, self._status_cb)
        self._pub_gripper_pos = rospy.Publisher(
            '/gripper_state', JointState, queue_size=10)

        # Initialize gripper state
        self._updated_gripper_pos = JointState()
        self._updated_gripper_pos.name = ['finger_joint']
        self._updated_gripper_pos.position = [0.0]

        # Start thread to publish gripper state
        self._pub_gripper_thread = threading.Thread(
            target=self._pub_pos_target)
        self._pub_gripper_thread.daemon = True
        self._pub_gripper_thread.start()

        # Wait for gripper connection
        self.wait_for_connection()

    def _status_cb(self, msg):
        """
        Callback function for gripper status subscriber.

        Args:
            msg (InputMsg): Gripper status message.
        """
        with self._get_state_lock:
            self.cur_status = msg
            # Update gripper position
            pos = self.get_pos()
            self._updated_gripper_pos.position[0] = pos

    def _pub_pos_target(self):
        """
        Function to run in background thread to publish updated
        gripper state.
        """
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            with self._pub_state_lock:
                self._pub_gripper_pos.publish(self._updated_gripper_pos)
            rate.sleep()

    def wait_for_connection(self, timeout=-1):
        """
        Waits for the gripper to establish connection.

        Args:
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if connection is established, False otherwise.
        """
        rospy.sleep(0.1)
        r = rospy.Rate(30)
        start_time = time.time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and time.time() - start_time > timeout):
                print_red('Gripper connection timed out.')
                return False
            if self.cur_status is not None:
                return True
            r.sleep()
        return False

    def activate(self, timeout=-1):
        """
        Activates the gripper.

        Args:
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if activation succeeded, False otherwise.
        """
        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = 0
        cmd.rSP = 255
        cmd.rFR = 150
        self._cmd_pub.publish(cmd)
        r = rospy.Rate(30)
        start_time = time.time()
        while not rospy.is_shutdown():
            if timeout >= 0. and time.time() - start_time > timeout:
                print_red('Gripper activation timed out.')
                return False
            if self.is_ready():
                return True
            r.sleep()
        return False

    def is_ready(self):
        """
        Checks if the gripper is ready.

        Returns:
            bool: True if gripper is ready, False otherwise.
        """
        with self._get_state_lock:
            return self.cur_status is not None and \
                   self.cur_status.gSTA == 3 and self.cur_status.gACT == 1

    def is_reset(self):
        """
        Checks if the gripper is reset.

        Returns:
            bool: True if gripper is reset, False otherwise.
        """
        with self._get_state_lock:
            return self.cur_status is None or \
                   self.cur_status.gSTA == 0 or self.cur_status.gACT == 0

    def is_moving(self):
        """
        Checks if the gripper is moving.

        Returns:
            bool: True if gripper is moving, False otherwise.
        """
        with self._get_state_lock:
            return self.cur_status.gGTO == 1 and self.cur_status.gOBJ == 0

    def is_stopped(self):
        """
        Checks if the gripper has stopped moving.

        Returns:
            bool: True if gripper is stopped, False otherwise.
        """
        with self._get_state_lock:
            return self.cur_status.gOBJ != 0

    def get_pos(self):
        """
        Gets the current position of the gripper.

        Returns:
            float: Gripper position in radians.
        """
        with self._get_state_lock:
            if self.cur_status is None:
                return 0.0
            po = self.cur_status.gPO
            # Convert position from gripper units to radians
            pos = np.clip((po - 230.) * (0.087 / (13. - 230.)), 0, 0.087)
            return pos

    def set_jpos(self, pos, vel=0.1, force=100, block=False, timeout=-1):
        """
        Sets the gripper position.

        Args:
            pos (float): Desired gripper position in radians.
            vel (float): Gripper speed in m/s. [0.013, 0.100]
            force (float): Gripper force in N. [30, 100]
            block (bool): Whether to block until movement is complete.
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if movement succeeded, False otherwise.
        """
        pos = clamp(pos, self.cfgs.EETOOL.OPEN_ANGLE, self.cfgs.EETOOL.CLOSE_ANGLE)
        # Convert position from radians to gripper units
        pr = int(np.clip((13. - 230.) / 0.087 * pos + 230., 0, 255))
        sp = int(np.clip(255. / (0.1 - 0.013) * (vel - 0.013), 0, 255))
        fr = int(np.clip(255. / (100. - 30.) * (force - 30.), 0, 255))

        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = pr
        cmd.rSP = sp
        cmd.rFR = fr
        self._cmd_pub.publish(cmd)
        rospy.sleep(0.1)
        if block:
            if not self.wait_until_moving(timeout):
                return False
            return self.wait_until_stopped(timeout)
        return True

    def open(self, vel=0.1, force=100, block=False, timeout=-1):
        """
        Opens the gripper.

        Args:
            vel (float): Gripper speed in m/s. [0.013, 0.100]
            force (float): Gripper force in N. [30, 100]
            block (bool): Whether to block until movement is complete.
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if movement succeeded, False otherwise.
        """
        return self.set_jpos(
            self.cfgs.EETOOL.OPEN_ANGLE, vel=vel, force=force,
            block=block, timeout=timeout)

    def close(self, vel=0.1, force=100, block=False, timeout=-1):
        """
        Closes the gripper.

        Args:
            vel (float): Gripper speed in m/s. [0.013, 0.100]
            force (float): Gripper force in N. [30, 100]
            block (bool): Whether to block until movement is complete.
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if movement succeeded, False otherwise.
        """
        return self.set_jpos(
            self.cfgs.EETOOL.CLOSE_ANGLE, vel=vel, force=force,
            block=block, timeout=timeout)

    def wait_until_stopped(self, timeout=-1):
        """
        Waits until the gripper has stopped moving.

        Args:
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if the gripper has stopped, False otherwise.
        """
        r = rospy.Rate(30)
        start_time = time.time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and time.time() - start_time > timeout) or self.is_reset():
                print_red('Gripper did not stop in time.')
                return False
            if self.is_stopped():
                return True
            r.sleep()
        return False

    def wait_until_moving(self, timeout=-1):
        """
        Waits until the gripper starts moving.

        Args:
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if the gripper started moving, False otherwise.
        """
        r = rospy.Rate(30)
        start_time = time.time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and time.time() - start_time > timeout) or self.is_reset():
                print_red('Gripper did not start moving in time.')
                return False
            if not self.is_stopped():
                return True
            r.sleep()
        return False

    def stop(self, block=False, timeout=-1):
        """
        Stops the gripper's movement.

        Args:
            block (bool): Whether to block until the gripper stops.
            timeout (float): Time to wait before giving up. Negative means wait indefinitely.

        Returns:
            bool: True if the gripper stopped, False otherwise.
        """
        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 0
        self._cmd_pub.publish(cmd)
        rospy.sleep(0.1)
        if block:
            return self.wait_until_stopped(timeout)
        return True

    def reset(self):
        """
        Resets the gripper to its initial state.
        """
        cmd = OutputMsg()
        cmd.rACT = 0
        self._cmd_pub.publish(cmd)

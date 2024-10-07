import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from gripper.gripper_server import GripperServer

class RobotiqGripperServerSim(GripperServer):
    def __init__(self):
        super().__init__()

        # Variable to store the gripper position
        self.gripper_pos = None

        # Subscribe to the gripper's joint states
        self.joint_state_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self._update_gripper,
            queue_size=1,
        )

        # Create an action client to control the gripper
        self.gripper_client = actionlib.SimpleActionClient(
            "/gripper_action_controller/gripper_cmd", GripperCommandAction
        )
        rospy.loginfo("Waiting for gripper action server...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Gripper action server connected.")

    def activate_gripper(self):
        self.gripper_command = self._generate_gripper_command("a", GripperCommandGoal())
        self._send_gripper_command(self.gripper_command)

    def reset_gripper(self):
        self.gripper_command = self._generate_gripper_command("r", GripperCommandGoal())
        self._send_gripper_command(self.gripper_command)
        self.activate_gripper()

    def open(self):
        self.gripper_command = self._generate_gripper_command("o", GripperCommandGoal())
        self._send_gripper_command(self.gripper_command)

    def close(self):
        self.gripper_command = self._generate_gripper_command("c", GripperCommandGoal())
        self._send_gripper_command(self.gripper_command)

    def move(self, position):
        self.gripper_command = self._generate_gripper_command(position, GripperCommandGoal())
        self._send_gripper_command(self.gripper_command)

    def _update_gripper(self, msg):
        """Callback to update the gripper's position from joint states."""
        # Replace 'gripper_finger_joint' with your actual gripper joint name
        gripper_joint_name = 'hande_left_finger_joint'
        try:
            if gripper_joint_name in msg.name:
                index = msg.name.index(gripper_joint_name)
                position = msg.position[index]
                self.gripper_pos = position
        except Exception as e:
            rospy.logerr(f"Error updating gripper position: {e}")


    def _generate_gripper_command(self, char, command):
        """Generate the gripper command based on input."""
        if char == "a":  # Activate
            command.command.position = 0.0  # Starting position (open)
            command.command.max_effort = 100.0  # Max effort

        elif char == "r":  # Reset or stop (open completely)
            command.command.position = 0.0
            command.command.max_effort = 0.0  # Resetting with no effort

        elif char == "c":  # Close fully
            command.command.position = 1.0  # Fully close
            command.command.max_effort = 100.0

        elif char == "o":  # Open fully
            command.command.position = 0.0  # Fully open
            command.command.max_effort = 100.0

        else:
            try:
                # Move to a specific position (interpreting char as position)
                position = float(char)
                command.command.position = max(0.0, min(1.0, position))  # Clamp between 0 and 1
                command.command.max_effort = 100.0
            except ValueError:
                pass

        return command

    def _send_gripper_command(self, command):
        """
        Send a gripper command to the robot through the action server.

        Args:
            command: The GripperCommandGoal message to send.
        """
        self.gripper_client.send_goal(command)
        self.gripper_client.wait_for_result()  # Optionally wait for the result

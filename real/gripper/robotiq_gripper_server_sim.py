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

        # Get min_angle and max_angle from ROS parameters
        self.stroke = 0.025

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
                # Normalize the position
                normalized_position = position / self.stroke
                normalized_position = round(normalized_position, 2)
                self.gripper_pos = normalized_position
        except Exception as e:
            rospy.logerr(f"Error updating gripper position: {e}")

    def _generate_gripper_command(self, char, command):
        """Generate the gripper command based on input."""
        try:
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
                # Move to a specific position (interpreting char as position)
                position = float(char)
                command.command.position = max(0.0, min(1.0, position))  # Clamp between 0 and 1
                command.command.max_effort = 100.0

        except ValueError:
            rospy.logwarn(f"Invalid position value provided: {char}")
        return command

    def _send_gripper_command(self, command):
        """
        Send a gripper command to the robot through the action server.

        Args:
            command: The GripperCommandGoal message to send.
        """
        rospy.loginfo("Sending goal to the gripper action server.")
        
        # Send the goal and wait for it to complete, with a timeout of 10 seconds
        self.gripper_client.send_goal_and_wait(command, rospy.Duration(10.0))
        
        # Check if the result was successful
        result = self.gripper_client.get_result()
        if result:
            rospy.loginfo(f"Gripper position: {self.gripper_pos}")
             #Print all attributes of the result
            rospy.loginfo("Gripper result details:")
            for attr in dir(result):
                if not attr.startswith('_') and not callable(getattr(result, attr)):
                    rospy.loginfo(f"{attr}: {getattr(result, attr)}")
        else:
            rospy.logwarn("Action did not finish before the timeout or was not successful.")
            self.gripper_client.cancel_goal()  # Cancel the goal if it didn't complete

    def cancel_all_goals(self):
        """Cancels all running goals on the action server."""
        rospy.loginfo("Canceling all goals on the action server.")
        self.gripper_client.cancel_all_goals()

    def stop_current_goal(self):
        """Stops tracking and cancels the current goal if it's still running."""
        rospy.loginfo("Canceling the current goal on the action server.")
        self.gripper_client.cancel_goal()

    def get_current_state(self):
        """Gets the current state of the action server for the active goal."""
        state = self.gripper_client.get_state()
        rospy.loginfo(f"Current action server state: {state}")
        return state

    def get_goal_status(self):
        """Gets the status text of the goal."""
        status_text = self.gripper_client.get_goal_status_text()
        rospy.loginfo(f"Goal status text: {status_text}")
        return status_text

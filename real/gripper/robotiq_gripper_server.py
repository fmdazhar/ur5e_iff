import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

from gripper.gripper_server import GripperServer

class RobotiqGripperServer(GripperServer):
    def __init__(self):
        super().__init__()

        self.gripper_state_sub = rospy.Subscriber(
            "Robotiq2FGripperRobotInput",
            inputMsg.Robotiq2FGripper_robot_input,
            self._update_gripper,
            queue_size=1,
        )
        self.gripperpub = rospy.Publisher(
            "Robotiq2FGripperRobotOutput",
            outputMsg.Robotiq2FGripper_robot_output,
            queue_size=1,
        )
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()

    def activate_gripper(self):
        self.gripper_command = self._generate_gripper_command("a", self.gripper_command)
        self._send_gripper_command(self.gripper_command)

    def reset_gripper(self):
        self.gripper_command = self._generate_gripper_command("r", self.gripper_command)
        self._send_gripper_command(self.gripper_command)
        self.activate_gripper()

    def open(self):
        self.gripper_command = self._generate_gripper_command("o", self.gripper_command)
        self._send_gripper_command(self.gripper_command)

    def close(self):
        self.gripper_command = self._generate_gripper_command("c", self.gripper_command)
        self._send_gripper_command(self.gripper_command)

    def move(self, position):
        self.gripper_command = self._generate_gripper_command(position, self.gripper_command)
        self._send_gripper_command(self.gripper_command)

    def _update_gripper(self, msg):
        """Internal callback to get the latest gripper position."""
        self.gripper_pos = msg.gPO

    def _generate_gripper_command(self, char, command):
        """Update the gripper command according to the character entered by the user."""
        if char == "a":
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP = 255
            command.rFR = 150

        elif char == "r":
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 0

        elif char == "c":
            command.rPR = 255

        elif char == "o":
            command.rPR = 0

        # If the command entered is an int, assign this value to rPR
        # (i.e., move to this position)
        else:
            try:
                command.rPR = int(char)
                if command.rPR > 255:
                    command.rPR = 255
                if command.rPR < 0:
                    command.rPR = 0
            except ValueError:
                pass
        return command

    def _send_gripper_command(self, command, mode="binary"):
        """
        Send a gripper command to the robot.

        Args:
            command: The gripper command message to send.
            mode (str): Mode of operation (not used in this implementation).
        """
        self.gripperpub.publish(command)

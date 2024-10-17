from abc import ABC, abstractmethod

class GripperServer(ABC):
    def __init__(self):
        self.gripper_pos = None  # Current position of the gripper

    @abstractmethod
    def activate_gripper(self):
        """Activate the gripper."""
        pass

    @abstractmethod
    def reset_gripper(self):
        """Reset the gripper to its default state."""
        pass

    @abstractmethod
    def open(self):
        """Open the gripper."""
        pass

    @abstractmethod
    def close(self):
        """Close the gripper."""
        pass

    @abstractmethod
    def move(self, position):
        """
        Move the gripper to a specific position.

        Args:
            position (float): Target position for the gripper.
        """
        pass

    @abstractmethod
    def _update_gripper(self, msg):
        """
        Internal callback to update the gripper's state.

        Args:
            msg: Message containing the gripper's state information.
        """
        pass

    @abstractmethod
    def _generate_gripper_command(self, command_input, command):
        """
        Generate a gripper command based on input.

        Args:
            command_input: Input to generate the command (e.g., 'o' for open).
            command: Existing command object to be modified.
        """
        pass

    @abstractmethod
    def _send_gripper_command(self, pos, mode="binary"):
        """
        Send a gripper command to the robot.

        Args:
            pos (float): Desired position or command for the gripper.
            mode (str): Mode of operation ('binary' or 'continuous').
        """
        pass

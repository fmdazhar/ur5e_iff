import numpy as np
from utils.input_utils import input2action  # Relative import for input2action
from devices.keyboard import Keyboard  # Relative import from devices.keyboard
from devices.spacemouse import SpaceMouse  # Relative import from devices.spacemouse


def test_keyboard():
    
    device = Keyboard(pos_sensitivity=1.0, rot_sensitivity=1.0)
    print("Testing Keyboard input... Press keys to see the output.")

    device.start_control()
    while True:
        # Get the input action from the keyboard
        action, grasp = input2action(device)

        # Print the current action (position and rotation inputs)
        print(f"Action: {action}, Grasp: {grasp}")


def test_spacemouse():
    
    device = SpaceMouse(pos_sensitivity=1.0, rot_sensitivity=1.0)
    print("Testing SpaceMouse input... Move the device to see the output.")

    device.start_control()
    while True:
        # Get the input action from the SpaceMouse
        action, grasp = input2action(device)

        # Print the current action (position and rotation inputs)
        print(f"Action: {action}, Grasp: {grasp}")


if __name__ == "__main__":
    # Set the test device you want to use (either 'test_keyboard' or 'test_spacemouse')
    # Uncomment the desired function to test

    # Test keyboard inputs
    test_keyboard()

    # Test SpaceMouse inputs
    # test_spacemouse()

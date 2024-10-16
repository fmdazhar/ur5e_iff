import os
from mujoco_sim.robots.gripper import Gripper

_HANDE_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/gripper/hande/hande.xml',
)

_JOINT = 'hande_left_finger_joint'

_ACTUATOR = 'hande_fingers_actuator'

class HANDE(Gripper):
    def __init__(self, name: str = None):
        super().__init__(_HANDE_XML, _JOINT, _ACTUATOR, name)
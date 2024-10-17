from cfgs.assets.default_configs import get_cfg_defaults
from cfgs.assets.robotiq_hande import get_robotiq_hande_cfg
from cfgs.assets.ur5e_arm import get_ur5e_arm_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = True

_C.ROBOT_DESCRIPTION = '/robot_description'

_C.ARM = get_ur5e_arm_cfg()

_C.EETOOL = get_robotiq_hande_cfg()


def get_cfg():
    return _C.clone()

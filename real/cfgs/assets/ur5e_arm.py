from yacs.config import CfgNode as CN

_C = CN()

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.CLASS = 'UR5e'
_C.MOVEGROUP_NAME = 'manipulator'
_C.ROSTOPIC_JOINT_STATES = '/joint_states'
_C.ROSTOPIC_WRENCH = '/cartesian_compliance_controller/ft_sensor_wrench'


# https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
_C.MAX_TORQUES = [150, 150, 150, 28, 28, 28]
_C.JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]
_C.LINK_NAMES = ['base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link',
                 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'flange', 'tool0']
# base frame for the arm
_C.ROBOT_BASE_FRAME = 'base_link'
_C.ROBOT_JOINT_TRAJECTORY_CONTROLLER = 'pos_joint_traj_controller'
# joint command topic
_C.ROBOT_JOINT_COMMAND_TOPIC = '/pos_joint_traj_controller/command'
# end-effector frame of the arm
_C.ROBOT_EE_FRAME = 'tcp_link'
_C.ROBOT_CARTESIAN_MOTION_CONTROLLER = 'cartesian_compliance_controller'
_C.ROBOT_EE_POSE_COMMAND_TOPIC = '/target_frame'

_C.JOINT_SPEED_TOPIC = '/joint_speed'
_C.URSCRIPT_TOPIC = '/ur_driver/URScript'
# inverse kinematics position tolerance (m)
_C.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.IK_ORIENTATION_TOLERANCE = 0.05
_C.RESET_POSITION = [0, -1.66, -1.92, -1.12, 1.57, 0]
_C.HOME_POSE = [0, 0, 0, 0, 0, 0, 0]
_C.MAX_JOINT_ERROR = 0.01
_C.MAX_JOINT_VEL_ERROR = 0.05
_C.MAX_EE_POS_ERROR = 0.001
# real part of the quaternion difference should be
# greater than 1-error
_C.MAX_EE_ORI_ERROR = 0.002
_C.TIMEOUT_LIMIT = 10

_C.RANDOM_XY_RANGE = 0.05  # Max random XY range during reset
_C.RANDOM_RZ_RANGE = 0.05  # Max random orientation reset range

_C.xyz_bounding_box_low = [-1.0, -1.0, 0.0]  # Min x, y, z coordinates
_C.xyz_bounding_box_high = [1.0, 1.0, 1.5]   # Max x, y, z coordinates
_C.rpy_bounding_box_low = [-3.14, -3.14, -3.14]  # Min roll, pitch, yaw
_C.rpy_bounding_box_high = [3.14, 3.14, 3.14]    # Max roll, pitch, yaw

_C.CONTROL_FREQUENCY = 10  # Used for interpolating between positions





def get_ur5e_arm_cfg():
    return _C.clone()

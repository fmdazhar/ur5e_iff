
"""
This file starts a control server running on the real time PC connected to the UR5e robot.
In a screen run `python main_server.py`
"""
from flask import Flask, request, jsonify
import numpy as np
import rospy
import time
import subprocess
from scipy.spatial.transform import Rotation as R
from absl import app, flags
from cfgs.ur5e_hande_cfg import get_cfg

import geometry_msgs.msg as geom_msg
from dynamic_reconfigure.client import Client as ReconfClient


FLAGS = flags.FLAGS
flags.DEFINE_list(
    "reset_joint_target",
    [0, 0, 0, -1.9, -0, 2, 0],
    "Target joint angles for the robot to reset to",
)


def main(_):
    ROS_PKG_NAME = "noetic_fake"
    RESET_JOINT_TARGET = FLAGS.reset_joint_target
    cfgs  = get_cfg()

    webapp = Flask(__name__)

    try:
        roscore = subprocess.Popen("roscore")
        time.sleep(1)
    except Exception as e:
        raise Exception("roscore not running", e)
    

    # Start ros node
    rospy.init_node("ur5e_control_api")

    # Start the combined roslaunch that controls both the robot and the gripper
    launch_command = [
            "roslaunch",
            ROS_PKG_NAME,
            "c_bot_fake.launch",
        ]
    
    try:
        ur5e_process = subprocess.Popen(launch_command, stdout=subprocess.PIPE)
        time.sleep(5)  # Wait for the launch to complete
    except Exception as e:
        raise Exception("Failed to launch UR5e with gripper", e)


    from arm.ur5e_ros import UR5eRealServer  # Assuming this file defines the UR5eRealServer class

    """Starts ur_driver controller"""
    robot_server = UR5eRealServer(
        cfgs=cfgs,
        reset_joint_target=RESET_JOINT_TARGET,
    )

    from gripper.robotiq_gripper_server import RobotiqGripperServer  # For Robotiq Gripper server (conditional)

    gripper_server = RobotiqGripperServer()
        
    # Initialize dynamic reconfigure client for the solver parameters
    solver_client = ReconfClient("/cartesian_compliance_controller/solver", timeout=30)
    
    # Initialize dynamic reconfigure client for PD gains
    pd_gains_client = ReconfClient("/cartesian_compliance_controller/pd_gains", timeout=30)

    # Initialize dynamic reconfigure client for PD gains
    stiffness_gains_client = ReconfClient("/cartesian_compliance_controller/stiffness", timeout=30)
    

    # Route for Getting End Effector Pose
    @webapp.route("/getposefull", methods=["POST"])
    def get_eef_pose():
        # Get the end-effector pose
        pos, quat, rot_mat, euler_ori = robot_server.get_ee_pose()

        # Return the pose data as a JSON response
        return jsonify({
            "position": pos.tolist(),
            "quaternion": quat.tolist(),
            "rotation_matrix": rot_mat.tolist(),
            "euler_angles": euler_ori.tolist()
        })
    
    # Route for Getting End Effector Pose (Position and Orientation)
    @webapp.route("/getpose", methods=["POST"])
    def get_pos():
        # Get the end-effector pose
        pos, quat, _, _ = robot_server.get_ee_pose()
        # Return the combined pose (position + orientation) as a JSON response
        return jsonify({"pose":  np.concatenate([pos, quat]).tolist()})


    @webapp.route("/geteefvel", methods=["POST"])
    def get_vel():
        # Get the end-effector velocity
        translational_vel, rotational_vel = robot_server.get_ee_vel()
        # Return the combined velocity as a JSON response
        return jsonify({"velocity": np.concatenate([translational_vel, rotational_vel]).tolist()})


    @webapp.route("/getforce", methods=["POST"])
    def get_force_torque():
        # Return the combined force and torque as a JSON response
        return jsonify({"force_torque": robot_server.get_force_torque()})

    @webapp.route("/getq", methods=["POST"])
    def get_q():
        return jsonify({"q": robot_server.get_jpos()})

    @webapp.route("/getdq", methods=["POST"])
    def get_dq():
        return jsonify({"q": robot_server.get_jvel()})

    @webapp.route("/getjacobian", methods=["POST"])
    def get_jacobian():
        return jsonify({"jacobian": np.array(robot_server.jacobian).tolist()})
    
    # Route for getting all state information
    @webapp.route("/getstate", methods=["POST"])
    def get_state():
        # Get the end-effector pose
        pos, quat, _, _ = robot_server.get_ee_pose()
        pose = np.concatenate([pos, quat]).tolist()
        # Get the end-effector velocity
        translational_vel, rotational_vel = robot_server.get_ee_vel()

        # Return the combined pose (position + orientation) as a JSON response
        return jsonify(
            {
                "pose": pose,
                "vel": np.concatenate([translational_vel, rotational_vel]).tolist(),
                "wrench": robot_server.get_wrench(),
                "q": robot_server.get_jpos(),
                "dq": robot_server.get_jvel(),
                "jacobian": robot_server.get_jacobian().tolist(),
                "gripper_pos": gripper_server.gripper_pos,
            }
        )

    # Route for Sending a pose command
    @webapp.route("/pose", methods=["POST"])
    def pose():
        pos = np.array(request.json["arr"])
        print("Moving to", pos)
        robot_server.set_ee_pose(pos)
        return "Moved"
    
    # Route for Sending a pose command
    @webapp.route("/pose_delta", methods=["POST"])
    def pose():
        delta_pos = np.array(request.json["arr"])
        print("Moving by", delta_pos)
        robot_server.move_ee_xyz(delta_pos)
        return "Moved"
    
    # Route for updating compliance parameters
    @webapp.route("/update_solver_param", methods=["POST"])
    def update_solver_param():
        solver_client.update_configuration(request.json)
        return "Updated compliance parameters"
    
    # Route for updating compliance parameters
    @webapp.route("/update_pd_gains_param", methods=["POST"])
    def update_pd_gains_param():
        pd_gains_client.update_configuration(request.json)
        return "Updated compliance parameters"
    
    # Route for updating compliance parameters
    @webapp.route("/update_stiffness_gains_param", methods=["POST"])
    def update_stiffness_gains_param():
        stiffness_gains_client.update_configuration(request.json)
        return "Updated compliance parameters"

    # Route for Running Joint Reset
    @webapp.route("/jointreset", methods=["POST"])
    def joint_reset():
        robot_server.reset_joint( use_urscript=False)
        return "Reset Joint"
    

    # Route for getting gripper distance
    @webapp.route("/get_gripper", methods=["POST"])
    def get_gripper():
        return jsonify({"gripper": gripper_server.gripper_pos})

    # Route for Activating the Gripper
    @webapp.route("/activate_gripper", methods=["POST"])
    def activate_gripper():
        print("activate gripper")
        gripper_server.activate_gripper()
        return "Activated"

    # Route for Resetting the Gripper. It will reset and activate the gripper
    @webapp.route("/reset_gripper", methods=["POST"])
    def reset_gripper():
        print("reset gripper")
        gripper_server.reset_gripper()
        return "Reset"

    # Route for Opening the Gripper
    @webapp.route("/open_gripper", methods=["POST"])
    def open():
        print("open")
        gripper_server.open()
        return "Opened"

    # Route for Closing the Gripper
    @webapp.route("/close_gripper", methods=["POST"])
    def close():
        print("close")
        gripper_server.close()
        return "Closed"

    # Route for moving the gripper
    @webapp.route("/move_gripper", methods=["POST"])
    def move_gripper():
        gripper_pos = request.json
        pos = np.clip(int(gripper_pos["gripper_pos"]), 0, 255)  # 0-255
        print(f"move gripper to {pos}")
        gripper_server.move(pos)
        return "Moved Gripper"
    
    @app.route('/shutdown', methods=['POST'])
    def shutdown():
        shutdown_func = request.environ.get('werkzeug.server.shutdown')
        if shutdown_func is None:
            raise RuntimeError('Not running the Werkzeug Server')
        shutdown_func()
        return 'Server shutting down...'

    webapp.run(host="0.0.0.0")

if __name__ == "__main__":
    app.run(main)



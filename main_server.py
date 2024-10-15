from flask import Flask, request, jsonify
import numpy as np
import os
import rospy
import time
import subprocess
from scipy.spatial.transform import Rotation as R
from absl import app, flags
from cfgs.ur5e_hande_cfg import get_cfg
import signal
import sys

import geometry_msgs.msg as geom_msg
from dynamic_reconfigure.client import Client as ReconfClient

FLAGS = flags.FLAGS
flags.DEFINE_list(
    "reset_joint_target",
    [0, -1.66, -1.92, -1.12, 1.57, 0],
    "Target joint angles for the robot to reset to",
)


def source_ros_setup(ROS_SETUP_BASH):
    import subprocess
    command = ['bash', '-c', f'source {ROS_SETUP_BASH} && env']  # Use f-string for interpolation
    proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    output = proc.communicate()[0]
    env = dict(os.environ)
    for line in output.decode().splitlines():
        (key, _, value) = line.partition("=")
        env[key] = value
    return env


def main(_):
    ROS_PKG_NAME = "noetic_fake"
    RESET_JOINT_TARGET = FLAGS.reset_joint_target
    cfgs = get_cfg()

    webapp = Flask(__name__)

    # Initialize variables to None
    roscore = None
    ur5e_process = None


    # Path to your ROS workspace and setup.bash
    ROS_SETUP_BASH = os.path.expanduser("~/Desktop/ws/ur5e_iff/catkin_ws/devel/setup.bash")

    # Source the ROS setup file and get the updated environment variables
    env = source_ros_setup(ROS_SETUP_BASH)

    # Function to handle SIGINT (Ctrl+C)
    def signal_handler(sig, frame):
        print('Ctrl+C pressed, shutting down.')
        if ur5e_process is not None:
            ur5e_process.terminate()
            ur5e_process.wait()
            print("UR5e process terminated.")
        if roscore is not None:
            roscore.terminate()
            roscore.wait()
            print("roscore terminated.")
        sys.exit(0)

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        try:
            # Start roscore
            roscore = subprocess.Popen("roscore", env=env)
            time.sleep(1)
        except Exception as e:
            raise Exception("roscore not running") from e

        # Start ROS node
        rospy.init_node("ur5e_control_api")

        # Start the combined roslaunch that controls both the robot and the gripper
        launch_command = ["roslaunch", ROS_PKG_NAME, "c_bot_fake.launch"]
        try:
            ur5e_process = subprocess.Popen(launch_command, stdout=subprocess.PIPE, env=env)
            time.sleep(5)  # Wait for the launch to complete
        except Exception as e:
            raise Exception("Failed to launch UR5e with gripper") from e

        # Import and initialize robot server
        from arm.ur5e_ros import UR5eRealServer  # Assuming this file defines the UR5eRealServer class
        robot_server = UR5eRealServer(cfgs=cfgs, reset_joint_target=RESET_JOINT_TARGET)

        # Import and initialize gripper server
        from gripper.robotiq_gripper_server_sim import RobotiqGripperServerSim
        gripper_server = RobotiqGripperServerSim()

        # Initialize dynamic reconfigure clients
        solver_client = ReconfClient("/cartesian_compliance_controller/solver")
        stiffness_gains_client = ReconfClient("/cartesian_compliance_controller/stiffness")
        forward_dynamics_client = ReconfClient("/cartesian_compliance_controller/solver/forward_dynamics")

        # Initialize dynamic reconfigure clients for PD gains
        pd_gains_clients = {
            "trans_x": ReconfClient("/cartesian_compliance_controller/pd_gains/trans_x"),
            "trans_y": ReconfClient("/cartesian_compliance_controller/pd_gains/trans_y"),
            "trans_z": ReconfClient("/cartesian_compliance_controller/pd_gains/trans_z"),
            "rot_x": ReconfClient("/cartesian_compliance_controller/pd_gains/rot_x"),
            "rot_y": ReconfClient("/cartesian_compliance_controller/pd_gains/rot_y"),
            "rot_z": ReconfClient("/cartesian_compliance_controller/pd_gains/rot_z")
        }

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
        def get_wrench():
            # Return the combined force and torque as a JSON response
            return jsonify({"force_torque": robot_server.get_wrench()})

        @webapp.route("/getq", methods=["POST"])
        def get_q():
            return jsonify({"q": robot_server.get_jpos()})

        @webapp.route("/getdq", methods=["POST"])
        def get_dq():
            return jsonify({"q": robot_server.get_jvel()})

        @webapp.route("/getjacobian", methods=["POST"])
        def get_jacobian():
            jpos = robot_server.get_jpos()
            return jsonify({"jacobian": np.array(robot_server.get_jacobian(jpos)).tolist()})
        
        # Route for getting all state information
        @webapp.route("/getstate", methods=["POST"])
        def get_state():
            # Get the end-effector pose
            jpos = robot_server.get_jpos()
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
                    "jacobian": robot_server.get_jacobian(jpos).tolist(),
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
        def pose_delta():
            delta_pos = np.array(request.json["arr"])
            print("Moving by", delta_pos)
            robot_server.move_ee_xyz(delta_pos)
            return "Moved"
        
        # Route for updating solver parameters
        @webapp.route("/update_solver_param", methods=["POST"])
        def update_solver_param():
            config = {key: float(value) for key, value in request.form.items()}  # Parse form data as floats
            solver_client.update_configuration(config)
            return "Updated solver parameters"

        # Route for updating stiffness gains parameters
        @webapp.route("/update_stiffness_gains_param", methods=["POST"])
        def update_stiffness_gains_param():
            config = {key: float(value) for key, value in request.form.items()}  # Parse form data as floats
            stiffness_gains_client.update_configuration(config)
            return "Updated stiffness gains parameters"

        # Route for updating forward dynamics parameters
        @webapp.route("/update_forward_dynamics_param", methods=["POST"])
        def update_forward_dynamics_param():
            config = {key: float(value) for key, value in request.form.items()}  # Parse form data as floats
            forward_dynamics_client.update_configuration(config)
            return "Updated forward dynamics parameters"

        # Route for updating PD gains parameters for a specific axis
        @webapp.route("/update_pd_gains_param/<axis>", methods=["POST"])
        def update_pd_gains_param(axis):
            if axis in pd_gains_clients:
                config = {key: float(value) for key, value in request.form.items()}  # Parse form data as floats
                pd_gains_clients[axis].update_configuration(config)
                return f"Updated PD gains for {axis}"
            else:
                return f"Invalid axis: {axis}", 400

        # Route for Running Joint Reset
        @webapp.route("/jointreset", methods=["POST"])
        def joint_reset():
            robot_server.reset_joint()
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
        
        # Route for shutting down the server and ROS processes
        @webapp.route("/shutdown", methods=["POST"])
        def shutdown():
            # Terminate the ROS processes
            print("Shutting down ROS processes from /shutdown route...")
            if ur5e_process is not None:
                ur5e_process.terminate()
                ur5e_process.wait()
                print("UR5e process terminated.")
            if roscore is not None:
                roscore.terminate()
                roscore.wait()
                print("roscore terminated.")

            # Shutdown the Flask server
            shutdown_server()
            return "Server shutting down..."

        def shutdown_server():
            func = request.environ.get('werkzeug.server.shutdown')
            if func is None:
                print("Not running with the Werkzeug Server")
                raise RuntimeError('Not running with the Werkzeug Server')
            func()

        webapp.run(host="0.0.0.0")

    except Exception as e:
        print(f"An exception occurred during initialization: {e}")
    finally:
        # Ensure that ROS processes are terminated
        print("Shutting down ROS processes...")
        if ur5e_process is not None:
            ur5e_process.terminate()
            ur5e_process.wait()
            print("UR5e process terminated.")
        if roscore is not None:
            roscore.terminate()
            roscore.wait()
            print("roscore terminated.")

if __name__ == "__main__":
    app.run(main)



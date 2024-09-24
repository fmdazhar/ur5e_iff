
"""
This file starts a control server running on the real time PC connected to the franka robot.
In a screen run `python franka_server.py`
"""
from flask import Flask, request, jsonify
import numpy as np
import rospy
import time
import subprocess
from scipy.spatial.transform import Rotation as R
from absl import app, flags

import geometry_msgs.msg as geom_msg
from dynamic_reconfigure.client import Client as ReconfClient

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "robot_ip", "172.16.0.2", "IP address of the franka robot's controller box"
)
flags.DEFINE_string(
    "gripper_ip", "192.168.1.114", "IP address of the robotiq gripper if being used"
)
flags.DEFINE_string(
    "gripper_type", "Robotiq", "Type of gripper to use: Robotiq, or None"
)
flags.DEFINE_list(
    "reset_joint_target",
    [0, 0, 0, -1.9, -0, 2, 0],
    "Target joint angles for the robot to reset to",
)


def main(_):
    ROS_PKG_NAME = "ur5e_franka_controllers"

    ROBOT_IP = FLAGS.robot_ip
    GRIPPER_IP = FLAGS.gripper_ip
    GRIPPER_TYPE = FLAGS.gripper_type
    RESET_JOINT_TARGET = FLAGS.reset_joint_target

    webapp = Flask(__name__)

    try:
        roscore = subprocess.Popen("roscore")
        time.sleep(1)
    except Exception as e:
        raise Exception("roscore not running", e)

    # Start ros node
    rospy.init_node("ur5e_control_api")

    """Starts ur_driver controller"""
    robot_server = UR5eRealServer(
        robot_ip=ROBOT_IP,
        gripper_type=GRIPPER_TYPE,
        ros_pkg_name=ROS_PKG_NAME,
        reset_joint_target=RESET_JOINT_TARGET,
    )

    reconf_client = ReconfClient(
        "cartesian_motion_controllerdynamic_reconfigure_compliance_param_node"
    )

    if GRIPPER_TYPE == "Robotiq":
        from robot_servers.robotiq_gripper_server import RobotiqGripperServer

        gripper_server = RobotiqGripperServer(gripper_ip=GRIPPER_IP)
        
    elif GRIPPER_TYPE == "None":
        pass
    else:
        raise NotImplementedError("Gripper Type Not Implemented")

    if __name__ == "__main__":
        app.run(main)



    # # Route for Starting ur_driver
    # @webapp.route("/startimp", methods=["POST"])
    # def start_ur_driver():
    #     robot_server.clear()
    #     robot_server.start_ur_driver()
    #     return "Started ur_driver"

    # # Route for Stopping ur_driver
    # @webapp.route("/stopimp", methods=["POST"])
    # def stop_ur_driver():
    #     robot_server.stop_ur_driver()
    #     return "Stopped ur_driver"

    # # Route for Getting Pose
    # @webapp.route("/getpos", methods=["POST"])
    # def get_pos():
    #     return jsonify({"pose": np.array(robot_server.pos).tolist()})

    # @webapp.route("/getpos_euler", methods=["POST"])
    # def get_pos_euler():
    #     r = R.from_quat(robot_server.pos[3:])
    #     euler = r.as_euler("xyz")
    #     return jsonify({"pose": np.concatenate([robot_server.pos[:3], euler]).tolist()})

    # @webapp.route("/getvel", methods=["POST"])
    # def get_vel():
    #     return jsonify({"vel": np.array(robot_server.vel).tolist()})

    # @webapp.route("/getforce", methods=["POST"])
    # def get_force():
    #     return jsonify({"force": np.array(robot_server.force).tolist()})

    # @webapp.route("/gettorque", methods=["POST"])
    # def get_torque():
    #     return jsonify({"torque": np.array(robot_server.torque).tolist()})

    # @webapp.route("/getq", methods=["POST"])
    # def get_q():
    #     return jsonify({"q": np.array(robot_server.q).tolist()})

    # @webapp.route("/getdq", methods=["POST"])
    # def get_dq():
    #     return jsonify({"dq": np.array(robot_server.dq).tolist()})

    # @webapp.route("/getjacobian", methods=["POST"])
    # def get_jacobian():
    #     return jsonify({"jacobian": np.array(robot_server.jacobian).tolist()})

    # # Route for getting gripper distance
    # @webapp.route("/get_gripper", methods=["POST"])
    # def get_gripper():
    #     return jsonify({"gripper": gripper_server.gripper_pos})

    # # Route for Running Joint Reset
    # @webapp.route("/jointreset", methods=["POST"])
    # def joint_reset():
    #     robot_server.clear()
    #     robot_server.reset_joint()
    #     return "Reset Joint"

    # # Route for Activating the Gripper
    # @webapp.route("/activate_gripper", methods=["POST"])
    # def activate_gripper():
    #     print("activate gripper")
    #     gripper_server.activate_gripper()
    #     return "Activated"

    # # Route for Resetting the Gripper. It will reset and activate the gripper
    # @webapp.route("/reset_gripper", methods=["POST"])
    # def reset_gripper():
    #     print("reset gripper")
    #     gripper_server.reset_gripper()
    #     return "Reset"

    # # Route for Opening the Gripper
    # @webapp.route("/open_gripper", methods=["POST"])
    # def open():
    #     print("open")
    #     gripper_server.open()
    #     return "Opened"

    # # Route for Closing the Gripper
    # @webapp.route("/close_gripper", methods=["POST"])
    # def close():
    #     print("close")
    #     gripper_server.close()
    #     return "Closed"

    # # Route for moving the gripper
    # @webapp.route("/move_gripper", methods=["POST"])
    # def move_gripper():
    #     gripper_pos = request.json
    #     pos = np.clip(int(gripper_pos["gripper_pos"]), 0, 255)  # 0-255
    #     print(f"move gripper to {pos}")
    #     gripper_server.move(pos)
    #     return "Moved Gripper"

    # # Route for Clearing Errors (Communcation constraints, etc.)
    # @webapp.route("/clearerr", methods=["POST"])
    # def clear():
    #     robot_server.clear()
    #     return "Clear"

    # # Route for Sending a pose command
    # @webapp.route("/pose", methods=["POST"])
    # def pose():
    #     pos = np.array(request.json["arr"])
    #     print("Moving to", pos)
    #     robot_server.move(pos)
    #     return "Moved"

    # # Route for getting all state information
    # @webapp.route("/getstate", methods=["POST"])
    # def get_state():
    #     return jsonify(
    #         {
    #             "pose": np.array(robot_server.pos).tolist(),
    #             "vel": np.array(robot_server.vel).tolist(),
    #             "force": np.array(robot_server.force).tolist(),
    #             "torque": np.array(robot_server.torque).tolist(),
    #             "q": np.array(robot_server.q).tolist(),
    #             "dq": np.array(robot_server.dq).tolist(),
    #             "jacobian": np.array(robot_server.jacobian).tolist(),
    #             "gripper_pos": gripper_server.gripper_pos,
    #         }
    #     )

    # # Route for updating compliance parameters
    # @webapp.route("/update_param", methods=["POST"])
    # def update_param():
    #     reconf_client.update_configuration(request.json)
    #     return "Updated compliance parameters"

    # webapp.run(host="0.0.0.0")



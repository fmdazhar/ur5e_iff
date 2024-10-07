import os
import subprocess
import time
from flask import Flask, jsonify, request

app = Flask(__name__)

roscore = None
ur5e_process = None

def launch_ur5e_simulation():
    """
    Function to launch UR5e simulation with ROS and Gazebo.
    """
    global roscore, ur5e_process
    try:
        # Start roscore if not already running
        print("Starting roscore...")
        roscore = subprocess.Popen("roscore")
        time.sleep(5)  # Wait for roscore to initialize

        # Launch UR5e simulation in Gazebo (or real environment if you modify the launch file)
        print("Launching UR5e robot in Gazebo...")
        launch_command = [
            "roslaunch", 
            "noetic_fake",  # Replace this with the appropriate ROS package name
            "c_bot_fake.launch"  # You can replace this with the appropriate launch file for your setup
        ]
        ur5e_process = subprocess.Popen(launch_command, stdout=subprocess.PIPE)
        time.sleep(5)  # Give it time to load

        print("UR5e Robot is successfully launched!")

    except Exception as e:
        print(f"Error launching UR5e: {e}")
    finally:
        if ur5e_process:
            print("UR5e process is running.")

@app.route('/')
def home():
    return jsonify({"message": "UR5e Simulation Flask Server is Running"}), 200

@app.route('/launch', methods=['POST'])
def launch():
    """
    Route to start the UR5e simulation.
    """
    if not ur5e_process:
        launch_ur5e_simulation()
        return jsonify({"message": "UR5e simulation launched"}), 200
    else:
        return jsonify({"message": "UR5e simulation is already running"}), 400

@app.route('/shutdown', methods=['POST'])
def shutdown():
    """
    Route to shut down the UR5e simulation and roscore.
    """
    global roscore, ur5e_process
    try:
        if ur5e_process:
            print("Shutting down UR5e simulation...")
            ur5e_process.terminate()
            ur5e_process.wait()
            ur5e_process = None

        if roscore:
            print("Shutting down roscore...")
            roscore.terminate()
            roscore.wait()
            roscore = None

        return jsonify({"message": "UR5e simulation and roscore shut down"}), 200

    except Exception as e:
        return jsonify({"message": f"Error during shutdown: {e}"}), 500


if __name__ == "__main__":
    # Launch the Flask server
    launch_ur5e_simulation()  # Optionally start UR5e when the server starts
    app.run(host='0.0.0.0', port=5000)

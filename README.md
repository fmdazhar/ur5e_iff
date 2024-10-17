# Noetic-UR5e

Docker file and docker-compose for ROS noetic supporting packages for the UR5e and Robotiq Hand-e gripper. The compose file starts two containers: the one running ROS and another running [noVNC](https://novnc.com/info.html) to use the GUI-based tools rviz and Gazebo.

## Installation Steps for Development

Follow these steps to set up your environment for UR5e robot development:

1. Install either [docker engine](https://docs.docker.com/engine/install/ubuntu/) (Linux) or [Docker Desktop](https://www.docker.com/).

2. **Clone the Repository**: Clone this repository to your local machine using the following command:

   ```bash
   git clone https://github.com/fmdazhar/ur5e_iff noetic_ur
   cd noetic_ur
   git submodule init
   git submodule update
   ```

3. **Install Visual Studio Code Extensions**: Install the Docker extension and Remote Development extension for Visual Studio Code.

4. Enable X11 forwarding

- Linux: `xhost +`
- Windows: use [VcXsrv](https://sourceforge.net/projects/vcxsrv/)

5. Build the Docker Container (first-time setup): Before running the container, you need to build it. Navigate to the root folder of the cloned repository and run the following command:

   ```bash
   docker compose build
   ```

6. **Run the Docker Container**: Navigate to the root folder of the cloned repository and run the following command to start the Docker container:

   ```bash
   docker compose up
   ```

7. **Attach Visual Studio Code**: Open Visual Studio Code, go to the Docker tab, right-click on the container, and select "Attach VSCode." This action will provide you with a Visual Studio Code environment within the container, preconfigured with ROS settings.

## Using the Docker Container

The Docker container sets up a ROS workspace in `/root/catkin_ws`.

There are three UR5e robots used in the lab. "a_bot" refers to the UR5e with a Robotiq 2F-85 Gripper, "b_bot" refers to the UR5e with the OnRobot RG2-FT Gripper, and "c_bot" refers to the UR5e with the Robotiq Hand-e Gripper.

Within this workspace, you'll find the custom configuration packages specific to running the robots in the lab:

- `catkin_ws/src/noetic_ur/noetic_description`: Contains the URDF of the UR5e robots and grippers in the lab. Macros are available for importing the robots into your own URDFs.
- `catkin_ws/src/noetic_ur/noetic_bringup`: Contains launch files for bringing up both "a_bot" and "b_bot," including ROS controllers config files and calibration files.
- `catkin_ws/src/noetic_ur/noetic_a_bot_moveit_config`: Generated `moveit_config` package for "a_bot."
- `catkin_ws/src/noetic_ur/noetic_b_bot_moveit_config`: Generated `moveit_config` package for "b_bot."
- `catkin_ws/src/noetic_ur/noetic_c_bot_moveit_config`: Generated `moveit_config` package for "c_bot."

The workspace also includes extra ROS package dependencies as git submodules in `catkin_ws`:

- `catkin_ws/src/controllers/cartesian_controllers`: Provides Cartesian motion, force, and compliance controllers for the `ros_control` framework.
- `catkin_ws/src/controllers/low_pass_force_torque_sensor_controller`: ROS package for controlling force-torque sensors via `ros_control` with an integrated low-pass filter.
- `catkin_ws/src/gazebo_pkgs/gazebo_gripper_action_controller`: ROS control gripper action controller for Gazebo.
- `catkin_ws/src/gazebo_pkgs/roboticsgroup_upatras_gazebo_plugins`: Contains the MimicJointPlugin, a simple model plugin for Gazebo to add mimic joint functionality.
- `catkin_ws/src/onrobot`: Contains ROS packages for controlling the OnRobot RG2-FT gripper.
- `catkin_ws/src/robotiq`: Contains ROS packages for controlling the Robotiq 2F-85 Gripper and Robotiq Hand-e Gripper.
- `catkin_ws/src/universal_robots/Universal_Robots_Client_Library`: A C++ library for accessing Universal Robots interfaces.
- `catkin_ws/src/universal_robots/Universal_Robots_ROS_Driver`: UR5e ROS driver.
- `catkin_ws/src/universal_robots/universal_robot`: ROS-Industrial Universal Robot meta-package. Contains UR5e URDF.

## How to Run Simulated Robots

Note that the same commands for "a_bot" also apply for "b_bot" and "c_bot"—just swap the names.

### Start the Simulated Robot in MoveIt!

```
roslaunch noetic_fake c_bot_fake.launch
```

### Start the Simulated Robot in Gazebo

```
roslaunch noetic_gazebo c_bot_gazebo.launch
```

### Running the Real Robot

Follow these steps to run the real robot:

1. **Set Up External Control URCap**: Follow the steps for setting up external control URCap and programming on the UR teach pendant. This only needs to be done once. Refer to [this guide](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap).

2. **Calibrate the Real Robot**: Calibrate the real robot using the following command, replacing `<c_bot robot ip>` with the actual IP of the "c_bot" robot:

   ```bash
   roslaunch noetic_bringup c_bot_calibration.launch robot_ip:=<c_bot robot ip>
   ```

3. **Start Up the Real Robot**: Launch the real robot using the following command, replacing `<c_bot robot ip>` with the actual IP of the "c_bot" robot and `<host computer ip>` with the IP of your host computer:

   ```bash
   roslaunch noetic_bringup c_bot_bringup.launch robot_ip:=<c_bot robot ip> reverse_ip:=<host computer ip>
   ```

4. **Run the External Control Program**: Run the external control program you have set up on the teach pendant.

### Running Different Controllers for the UR5e

There are various controllers available for the robot as part of ROS control. You can find their configurations in `catkin_ws/src/noetic_ur/noetic_bringup/config/c_bot_controllers.yaml` 

To work with different controllers:

1. Make sure the real robot is running with ROS.

2. Open up `rqt_gui` using the command: `rosrun rqt_gui rqt_gui`.

3. In `rqt_gui`, go to `Plugins` -> `Robot Tools` -> `Controller Manager`.

4. Select the namespace of the controller manager.

5. To switch to a different controller, select the desired controller from the drop-down menu or radio buttons provided in the plugin interface.

   **Note**: Before switching to a different controller, ensure that the current running controller, if it shares resources, is turned off to avoid conflicts.

6. In `rqt_gui`, go to `Plugins` -> `Configuration` -> `Dynamic Reconfigure`. Experiment with different configuration values of the controllers.

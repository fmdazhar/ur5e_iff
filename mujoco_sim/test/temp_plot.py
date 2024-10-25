import time
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt

from mujoco_sim import envs

env = envs.ur5ePickCubeGymEnv(action_scale=(1, 1))
action_spec = env.action_space

# Access the controller in the environment
controller = env.controller

def sample():
    a = np.random.uniform(action_spec.low, action_spec.high, action_spec.shape)
    return a.astype(action_spec.dtype)

m = env.model
d = env.data

reset = False
KEY_SPACE = 32
action = sample()  # Generate an initial action sample
last_sample_time = time.time()  # Track the last sample time

def key_callback(keycode):
    if keycode == KEY_SPACE:
        global reset
        reset = True

# # Set initial parameters for the controller
# controller.set_parameters(
#     damping_ratio=1.0,  # Initial damping ratio
#     error_tolerance_pos=0.005,
#     error_tolerance_ori=0.005,
#     max_pos_acceleration=2.0,
#     max_ori_acceleration=2.0,
#     max_angvel=0.5,
#     pos_gains=(100, 100, 100),
#     ori_gains=(50, 50, 50),
#     pos_kd=(20, 20, 20),
#     ori_kd=(10, 10, 10),
#     method="dynamics"
# )

# Set up plotting
fig, axs = plt.subplots(4, 1, figsize=(10, 20))
axs[0].set_title("End-Effector Target vs Current Position (X Coordinate)")
axs[1].set_title("End-Effector Target vs Current Position (Y Coordinate)")
axs[2].set_title("End-Effector Target vs Current Position (Z Coordinate)")
axs[3].set_title("Grasping DOF")
for ax in axs:
    ax.set_xlabel("Time Step")
    ax.set_ylabel("Position (m)")

# Data for plotting
time_steps = []
current_positions_x = []
target_positions_x = []
current_positions_y = []
target_positions_y = []
current_positions_z = []
target_positions_z = []
gripper_states = []

env.reset()
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    start = time.time()
    step_count = 0
    while viewer.is_running():
        if reset:
            env.reset()
            action = sample()  # Generate a new action sample
            last_sample_time = time.time()  # Reset the action timer
            reset = False
        else:
            step_start = time.time()

            # Update the action every 5 seconds
            if time.time() - last_sample_time >= 5.0:
                action = sample()  # Generate a new action sample
                last_sample_time = time.time()  # Update the last sample time

            # Control and observe the target vs. current position
            target_pos = d.mocap_pos[0]
            current_pos = d.site_xpos[controller.site_id]
            gripper_state = d.qpos[controller.dof_ids[-1]]  # Assuming the last DOF is for gripper
            # print(f"Target position: {target_pos}, Current position: {current_pos}, Gripper state: {gripper_state}")

            # Log data for plotting
            time_steps.append(step_count)
            target_positions_x.append(target_pos[0])
            current_positions_x.append(current_pos[0])
            target_positions_y.append(target_pos[1])
            current_positions_y.append(current_pos[1])
            target_positions_z.append(target_pos[2])
            current_positions_z.append(current_pos[2])
            gripper_states.append(gripper_state)

            # Update plot every 10 steps
            if step_count % 10 == 0:
                axs[0].clear()
                axs[0].set_title("End-Effector Target vs Current Position (X Coordinate)")
                axs[0].set_xlabel("Time Step")
                axs[0].set_ylabel("Position (m)")
                axs[0].plot(time_steps, target_positions_x, label='Target Position X', color='g')
                axs[0].plot(time_steps, current_positions_x, label='Current Position X', color='b')
                axs[0].legend()

                axs[1].clear()
                axs[1].set_title("End-Effector Target vs Current Position (Y Coordinate)")
                axs[1].set_xlabel("Time Step")
                axs[1].set_ylabel("Position (m)")
                axs[1].plot(time_steps, target_positions_y, label='Target Position Y', color='g')
                axs[1].plot(time_steps, current_positions_y, label='Current Position Y', color='b')
                axs[1].legend()

                axs[2].clear()
                axs[2].set_title("End-Effector Target vs Current Position (Z Coordinate)")
                axs[2].set_xlabel("Time Step")
                axs[2].set_ylabel("Position (m)")
                axs[2].plot(time_steps, target_positions_z, label='Target Position Z', color='g')
                axs[2].plot(time_steps, current_positions_z, label='Current Position Z', color='b')
                axs[2].legend()

                axs[3].clear()
                axs[3].set_title("Grasping DOF")
                axs[3].set_xlabel("Time Step")
                axs[3].set_ylabel("Gripper State")
                axs[3].plot(time_steps, gripper_states, label='Gripper State', color='r')
                axs[3].legend()

                plt.pause(0.01)

            env.step(action)
            viewer.sync()
            time_until_next_step = env.control_dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            step_count += 1

plt.show()
plt.close(fig)
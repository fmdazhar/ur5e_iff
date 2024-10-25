import time
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # If you're using a system with full GUI support
import matplotlib.pyplot as plt
import multiprocessing
from dm_robotics.transformations import transformations as tr
from mujoco_sim import envs

# Setup environment
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

def plot_data(queue):
    fig, axs = plt.subplots(10, 1, figsize=(10, 50))
    plt.ion()  # Turn on interactive mode for live plotting

    for ax in axs:
        ax.set_xlabel("Time Step")
        ax.set_ylabel("Value")
    # Initialize data lists outside the loop
    time_steps = []
    target_positions_x = []
    current_positions_x = []
    target_positions_y = []
    current_positions_y = []
    target_positions_z = []
    current_positions_z = []
    target_orientations = []
    current_orientations = []
    gripper_states = []
    joint_positions = []
    joint_velocities = []
    while True:
        if not queue.empty():
            batch_data = queue.get()  # Get the entire batch of data
            for data in batch_data:
                # data = queue.get()
                (step_count, target_pos, current_pos,
                target_orientation, current_orientation,
                gripper_state, current_joint_positions, current_joint_velocities) = data
                # Accumulate data
                time_steps.append(step_count)
                target_positions_x.append(target_pos[0])
                current_positions_x.append(current_pos[0])
                target_positions_y.append(target_pos[1])
                current_positions_y.append(current_pos[1])
                target_positions_z.append(target_pos[2])
                current_positions_z.append(current_pos[2])
                target_orientations.append(target_orientation)
                current_orientations.append(current_orientation)
                gripper_states.append(gripper_state)
                joint_positions.append(current_joint_positions)
                joint_velocities.append(current_joint_velocities)
                # Limit the data lists to the last 50 points
                max_length = 50
                if len(time_steps) > max_length:
                    time_steps = time_steps[-max_length:]
                    target_positions_x = target_positions_x[-max_length:]
                    current_positions_x = current_positions_x[-max_length:]
                    target_positions_y = target_positions_y[-max_length:]
                    current_positions_y = current_positions_y[-max_length:]
                    target_positions_z = target_positions_z[-max_length:]
                    current_positions_z = current_positions_z[-max_length:]
                    target_orientations = target_orientations[-max_length:]
                    current_orientations = current_orientations[-max_length:]
                    gripper_states = gripper_states[-max_length:]
                    joint_positions = joint_positions[-max_length:]
                    joint_velocities = joint_velocities[-max_length:]

                # Plot the data
                axs[0].clear()
                axs[0].set_title("End-Effector Target vs Current Position (X Coordinate)")
                axs[0].plot(time_steps, target_positions_x, label='Target Position X', color='g')
                axs[0].plot(time_steps, current_positions_x, label='Current Position X', color='b')
                axs[0].legend()

                axs[1].clear()
                axs[1].set_title("End-Effector Target vs Current Position (Y Coordinate)")
                axs[1].plot(time_steps, target_positions_y, label='Target Position Y', color='g')
                axs[1].plot(time_steps, current_positions_y, label='Current Position Y', color='b')
                axs[1].legend()

                axs[2].clear()
                axs[2].set_title("End-Effector Target vs Current Position (Z Coordinate)")
                axs[2].plot(time_steps, target_positions_z, label='Target Position Z', color='g')
                axs[2].plot(time_steps, current_positions_z, label='Current Position Z', color='b')
                axs[2].legend()

                axs[3].clear()
                axs[3].set_title("End-Effector Target vs Current Orientation (Quaternion W)")
                axs[3].plot(time_steps, [ori[0] for ori in target_orientations], label='Target Orientation W', color='g')
                axs[3].plot(time_steps, [ori[0] for ori in current_orientations], label='Current Orientation W', color='b')
                axs[3].legend()

                axs[4].clear()
                axs[4].set_title("End-Effector Target vs Current Orientation (Quaternion X)")
                axs[4].plot(time_steps, [ori[1] for ori in target_orientations], label='Target Orientation X', color='g')
                axs[4].plot(time_steps, [ori[1] for ori in current_orientations], label='Current Orientation X', color='b')
                axs[4].legend()

                axs[5].clear()
                axs[5].set_title("End-Effector Target vs Current Orientation (Quaternion Y)")
                axs[5].plot(time_steps, [ori[2] for ori in target_orientations], label='Target Orientation Y', color='g')
                axs[5].plot(time_steps, [ori[2] for ori in current_orientations], label='Current Orientation Y', color='b')
                axs[5].legend()

                axs[6].clear()
                axs[6].set_title("End-Effector Target vs Current Orientation (Quaternion Z)")
                axs[6].plot(time_steps, [ori[3] for ori in target_orientations], label='Target Orientation Z', color='g')
                axs[6].plot(time_steps, [ori[3] for ori in current_orientations], label='Current Orientation Z', color='b')
                axs[6].legend()

                axs[7].clear()
                axs[7].set_title("Gripper State (Target vs Current)")
                axs[7].plot(time_steps, gripper_states, label='Gripper State', color='r')
                axs[7].legend()

                axs[8].clear()
                axs[8].set_title("Joint Positions (q) for All Joints")
                for i in range(6):
                    axs[8].plot(time_steps, [jp[i] for jp in joint_positions], label=f'Joint {i+1}')
                axs[8].legend()

                axs[9].clear()
                axs[9].set_title("Joint Velocities (dq) for All Joints")
                for i in range(6):
                    axs[9].plot(time_steps, [jv[i] for jv in joint_velocities], label=f'Joint {i+1}')
                axs[9].legend()

                plt.pause(0.01)


def run_simulation(queue):
    global reset
    global last_sample_time
    global action
    env.reset()
    step_count = 0
    BATCH_SIZE = 10  # Send data in batches of 10 time steps
    data_buffer = []
    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
        while viewer.is_running():
            if reset:
                env.reset()
                action = sample()
                last_sample_time = time.time()
                reset = False
            else:
                step_start = time.time()
                if time.time() - last_sample_time >= 3.0:
                    action = sample()
                    last_sample_time = time.time()

                target_pos = d.mocap_pos[0]
                current_pos = d.site_xpos[controller.site_id]
                current_orientation = tr.mat_to_quat(d.site_xmat[controller.site_id].reshape((3, 3)))
                target_orientation = d.mocap_quat[0]
                current_joint_positions = d.qpos[controller.dof_ids].copy()
                current_joint_velocities = d.qvel[controller.dof_ids].copy()
                gripper_state = d.ctrl[env._gripper_ctrl_id]

                # Accumulate data
                data_buffer.append((step_count, target_pos, current_pos,
                                    target_orientation, current_orientation,
                                    gripper_state, current_joint_positions, current_joint_velocities))

                # Send data in batches
                if len(data_buffer) >= BATCH_SIZE:
                    queue.put(data_buffer)
                    data_buffer = []

                env.step(action)
                viewer.sync()
                time_until_next_step = env.control_dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                step_count += 1


# Setup multiprocessing
if __name__ == "__main__":
    # Create a multiprocessing queue
    data_queue = multiprocessing.Queue()

    # Start the plotting process
    plot_process = multiprocessing.Process(target=plot_data, args=(data_queue,), daemon=True)
    plot_process.start()

    # Run the simulation in the main process
    run_simulation(data_queue)

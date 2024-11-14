import mujoco
import mujoco.viewer as mjv
import os
import numpy as np
import pygame

# Load the model XML file
xml_path = "cp.xml"
if not os.path.exists(xml_path):
    raise FileNotFoundError(f"XML file '{xml_path}' does not exist.")

# Load the model and create a simulation
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Define movement step size
move_step = 0.000001

# Initialize pygame for key handling
pygame.init()
screen = pygame.display.set_mode((100, 100))  # Dummy window for capturing input

def handle_keyboard_input():
    connector_joint_id = model.joint('connector').id
    current_pos = data.jnt('connector').qpos.copy()  # Extract current position
    # print(current_pos)

    # Handle key presses
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:  # Move up
        current_pos[2] += move_step
    if keys[pygame.K_s]:  # Move down
        current_pos[2] -= move_step
    if keys[pygame.K_a]:  # Move left
        current_pos[1] -= move_step
    if keys[pygame.K_d]:  # Move right
        current_pos[1] += move_step
    if keys[pygame.K_q]:  # Move forward
        current_pos[0] += move_step
    if keys[pygame.K_e]:  # Move backward
        current_pos[0] -= move_step

    # Update position in qpos
    data.jnt("connector").qpos = current_pos
    data.qvel[connector_joint_id] = 0  # Reset velocity
    data.qacc[connector_joint_id] = 0  # Reset acceleration
    data.qfrc_applied[connector_joint_id] = 0  # Reset any applied forces
    mujoco.mj_forward(model, data)  # Update simulation state after position change

# Set up the viewer
def main():
    with mjv.launch_passive(model, data) as viewer:
        running = True
        while viewer.is_running() and running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            handle_keyboard_input()  # Handle key presses
            mujoco.mj_step(model, data)  # Step the simulation
            viewer.sync()

    pygame.quit()  # Clean up pygame

if __name__ == "__main__":
    main()

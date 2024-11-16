import mujoco
import mujoco.viewer as mujoco_viewer
import os

# Load the model XML file
xml_path = "port_changed.xml"
if not os.path.exists(xml_path):
    raise FileNotFoundError(f"XML file '{xml_path}' does not exist.")

# Load the MuJoCo model from the XML file
model = mujoco.MjModel.from_xml_path(xml_path)

# Create a data object for the model
data = mujoco.MjData(model)

# Create a viewer to visualize the model
viewer = mujoco_viewer.launch_passive(model, data)

# Run the simulation loop
while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()

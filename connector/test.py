import mujoco
import mujoco.viewer as mjv
import os

# Load the model XML file
xml_path = "conn.xml"
if not os.path.exists(xml_path):
    raise FileNotFoundError(f"XML file '{xml_path}' does not exist.")

# Load the model and create a simulation
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Set up the viewer
def main():
    with mjv.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()

import os
import trimesh
import xml.etree.ElementTree as ET

# Paths
mesh_dir = "port1"
xml_file = "port.xml"
output_file = "port_changed.xml"

# Scale factor from the XML file
scale_factor = 0.001

# Explicit mapping for port2 files with hyphens in filenames
port2_mapping = {
    "port2_top": "port2-Plate_Top",
    "port2_bottom": "port2-Plate_Bottom",
    "port2_left1": "port2-Plate_Left1",
    "port2_left2": "port2-Plate_Left2",
    "port2_right1": "port2-Plate_Right1",
    "port2_right2": "port2-Plate_Right2",
}
connector1_mapping = {
    "connector1_front": "connector1_front",
    "connector1_back": "connector1_back",
    "connector1_center": "connector1_center",
    "connector1_bottom": "connector1_bottom",
}  

def compute_bounding_box(mesh_path):
    mesh = trimesh.load_mesh(mesh_path)
    bounding_box = mesh.bounding_box
    # Scale the bounding box size and position by scale_factor
    size = bounding_box.extents / 2 * scale_factor  # half-extents for MuJoCo box size
    pos = bounding_box.centroid * scale_factor      # center position for MuJoCo box position
    return size, pos

def replace_all_meshes_with_boxes(xml_file, output_file, mesh_dir):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    mesh_data = {}

    # Process all .stl files in mesh_dir and add them to mesh_data with appropriate names
    for filename in os.listdir(mesh_dir):
        if filename.endswith(".stl"):
            mesh_name = os.path.splitext(filename)[0]
            mesh_path = os.path.join(mesh_dir, filename)
            size, pos = compute_bounding_box(mesh_path)
            mesh_data[mesh_name] = {"size": size, "pos": pos}
            print(f"Processed mesh '{mesh_name}' with scaled size {size} and scaled position {pos}")

    # Iterate over all <geom> elements in the XML and replace with boxes
    for geom in root.findall(".//geom"):
        mesh_name = geom.get("mesh")
        if mesh_name:
            # Check if mesh_name is in port2_mapping and use the mapped name if it is
            actual_mesh_name = port2_mapping.get(mesh_name, mesh_name)
            if actual_mesh_name in mesh_data:
                del geom.attrib["mesh"]
                geom.set("type", "box")
                size_str = " ".join(map(str, mesh_data[actual_mesh_name]["size"]))
                pos_str = " ".join(map(str, mesh_data[actual_mesh_name]["pos"]))
                geom.set("size", size_str)
                geom.set("pos", pos_str)
                print(f"Replaced mesh '{mesh_name}' in XML with a scaled box of size {mesh_data[actual_mesh_name]['size']} at position {mesh_data[actual_mesh_name]['pos']}")
            else:
                print(f"Mesh '{mesh_name}' referenced in XML but not found in '{mesh_dir}'")

    tree.write(output_file)
    print(f"Updated XML saved to {output_file}")

# Run the function
replace_all_meshes_with_boxes(xml_file, output_file, mesh_dir)

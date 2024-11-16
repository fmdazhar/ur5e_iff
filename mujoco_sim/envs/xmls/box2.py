import trimesh

# Load the .stl file
# mesh = trimesh.load_mesh("connector1/connector1_back.stl")
# mesh = trimesh.load_mesh("connector1/connector1_bottom.stl")
mesh = trimesh.load_mesh("connector1/connector1_center.stl")



# Compute the bounding box of the mesh
bounding_box = mesh.bounding_box

# Get the size of the box (half-extents for MuJoCo)
box_size = bounding_box.extents / 2  # MuJoCo requires half extents for the `size` attribute

# Find the center position of the bounding box
box_center_position = bounding_box.centroid

print(f"Box Size (half-extents for MuJoCo): {box_size}")
print(f"Box Center Position: {box_center_position}")

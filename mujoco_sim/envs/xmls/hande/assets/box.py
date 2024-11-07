import trimesh
import numpy as np

# Load the mesh
mesh = trimesh.load('finger_2.stl')


# Calculate the 90th percentile in the Z direction to get the top 10%
z_threshold = np.percentile(mesh.vertices[:, 2], 97)

# Identify vertices in the top 10%
top_10_percent_vertices = mesh.vertices[:, 2] > z_threshold

# Identify faces where all vertices are in the top 10%
faces_in_top_10_percent = np.all(top_10_percent_vertices[mesh.faces], axis=1)

# Create the submesh using the face mask
top_10_percent_mesh_list = mesh.submesh([faces_in_top_10_percent], only_watertight=False)

# Check if there are submeshes and use the first one
if top_10_percent_mesh_list:
    top_10_percent_mesh = top_10_percent_mesh_list[0]  # Access the first submesh
    bounding_box = top_10_percent_mesh.bounding_box.extents
    centroid = top_10_percent_mesh.centroid

    print("Bounding box size (half-dimensions):", bounding_box / 2)
    print("Centroid position:", centroid)
else:
    print("No submesh found in the top 10% of the mesh.")
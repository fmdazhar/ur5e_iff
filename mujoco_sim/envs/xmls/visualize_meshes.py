import trimesh

# Load and visualize each mesh separately
connector_back = trimesh.load_mesh('connector1/connector1_back.stl')
# connector_center = trimesh.load_mesh('connector1/connector1_center.stl')
# connector_bottom = trimesh.load_mesh('connector1/connector1_bottom.stl')

# Display each mesh separately
connector_back.show(title='Connector Back')
# connector_center.show(title='Connector Center')
# connector_bottom.show(title='Connector Bottom')

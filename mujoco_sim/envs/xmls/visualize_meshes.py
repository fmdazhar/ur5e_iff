import trimesh
import os
import time

def visualize_stl_files_in_folder(folder_path, delay=2):
    # List all .stl files in the specified folder
    stl_files = [f for f in os.listdir(folder_path) if f.endswith('.stl')]
    
    # Sort files alphabetically (optional)
    stl_files.sort()

    for stl_file in stl_files:
        # Construct full file path
        file_path = os.path.join(folder_path, stl_file)
        
        # Load the STL file
        mesh = trimesh.load_mesh(file_path)
        
        # Display the mesh with title
        print(f"Displaying: {stl_file}")
        mesh.show(title=stl_file)
        

if __name__ == "__main__":
    folder_path = "port1"  # Replace with your folder path
    visualize_stl_files_in_folder(folder_path)



# import trimesh

# # Load and visualize each mesh separately
# port1 = trimesh.load_mesh('port1/port1_back1.stl')

# # Display each mesh separately
# port1.show(title='port1')

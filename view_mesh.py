import open3d as o3d

def visualize_meshes_in_separate_windows(mesh_file_1, mesh_file_2):
    # Load the first mesh
    print(f"Loading mesh: {mesh_file_1}")
    mesh1 = o3d.io.read_point_cloud(mesh_file_1)
    if mesh1.is_empty():
        print(f"[ERROR] The file {mesh_file_1} could not be loaded or is empty.")
        return

    # Load the second mesh
    print(f"Loading mesh: {mesh_file_2}")
    mesh2 = o3d.io.read_point_cloud(mesh_file_2)
    if mesh2.is_empty():
        print(f"[ERROR] The file {mesh_file_2} could not be loaded or is empty.")
        return

    # First window for mesh1
    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name="Mesh 1", width=800, height=600, left=100, top=100)
    vis1.add_geometry(mesh1)

    # Second window for mesh2
    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name="Mesh 2", width=800, height=600, left=1000, top=100)  # Position second window to the right
    vis2.add_geometry(mesh2)

    # Render both windows
    vis1.poll_events()
    vis1.update_renderer()

    vis2.poll_events()
    vis2.update_renderer()

    print("Press 'q' in both windows to exit.")
    vis1.run()  # Run first window
    vis2.run()  # Run second window

    # Destroy windows
    vis1.destroy_window()
    vis2.destroy_window()
if __name__ == "__main__":
    # Define the input file paths
    mesh_file_1 = "data/mesh_output/mesh_model.ply"  
    mesh_file_2 = "data/merge_output/final_cleaned_model.pcd" 
    
    visualize_meshes_in_separate_windows(mesh_file_1, mesh_file_2)

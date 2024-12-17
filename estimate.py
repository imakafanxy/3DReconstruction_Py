import open3d as o3d
import numpy as np

def calculate_distance(points):
    """
    Calculate the distance between two selected points and convert it to centimeters.
    """
    if len(points) != 2:
        print("Error: Exactly two points must be selected!")
        return None

    point1 = np.array(points[0])
    point2 = np.array(points[1])
    distance = np.linalg.norm(point1 - point2)  # Distance in meters
    distance_cm = distance * 100  # Convert to centimeters
    print(f"Selected Points: {point1}, {point2}")
    print(f"Distance between points: {distance_cm:.2f} cm")
    return distance_cm

def main():
    # Load the point cloud or mesh
    file_path = "data/merge_output/final_cleaned_model.pcd"  # Set your file path here
    geometry = o3d.io.read_triangle_mesh(file_path) if file_path.endswith(".ply") else o3d.io.read_point_cloud(file_path)

    if geometry.is_empty():
        print("Error: Unable to load the file or file is empty.")
        return

    print("Instruction: Select two points in the viewer, and the program will calculate the distance between them.")
    print("Left-click to select points. Press 'q' to close the viewer after selecting two points.")

    # Enable visualizer with point picking
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Select Two Points")
    vis.add_geometry(geometry)
    vis.run()  # User selects points
    vis.destroy_window()

    # Retrieve selected points
    selected_points = vis.get_picked_points()
    if len(selected_points) != 2:
        print(f"Error: {len(selected_points)} points were selected. Please select exactly two points.")
        return

    # Extract coordinates from the point indices
    geometry_points = np.asarray(geometry.vertices if isinstance(geometry, o3d.geometry.TriangleMesh) else geometry.points)
    selected_coordinates = [geometry_points[i] for i in selected_points]

    # Calculate and display the distance
    calculate_distance(selected_coordinates)

if __name__ == "__main__":
    main()

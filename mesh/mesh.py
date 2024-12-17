import open3d as o3d
import numpy as np


def analyze_and_highlight_density(pcd, density_threshold=0.5):
    """
    Analyze point density and highlight low-density points in red,
    while keeping original colors for the rest.
    """
    print("Estimating normals for density analysis...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    print("Calculating point densities...")
    distances = pcd.compute_nearest_neighbor_distance()
    densities = 1.0 / (np.asarray(distances) + 1e-6)  # Avoid division by zero

    print(f"Highlighting points with density < {density_threshold}...")
    low_density_mask = densities < density_threshold
    high_density_mask = ~low_density_mask

    # Create a copy of the original colors
    if pcd.has_colors():
        colors = np.asarray(pcd.colors).copy()
    else:
        # If no colors exist, initialize with white
        colors = np.ones((len(pcd.points), 3))

    # Set low-density points to red
    colors[low_density_mask] = [1.0, 0.0, 0.0]  # Red color for low-density points

    # Apply the updated colors to the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Original points: {len(pcd.points)}, Low-density points: {np.sum(low_density_mask)}")
    return pcd, high_density_mask



def preprocess_pcd(pcd, nb_neighbors=50, std_ratio=4.0):
    print("Removing outliers using Statistical Outlier Removal...")
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return pcd


def fill_mesh_holes(mesh):
    """
    Fill small holes in the mesh.
    """
    print("Filling holes in the mesh...")
    mesh.compute_adjacency_list()  # Ensure adjacency is computed
    filled_mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
    print("Hole filling complete.")
    return filled_mesh

def poisson_mesh(pcd, depth=12, scale=1.5):
    print("Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=100))
    pcd.orient_normals_consistent_tangent_plane(k=100)

    print("Performing Poisson Surface Reconstruction...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, scale=scale
    )

    return mesh


def smooth_mesh(mesh, laplacian_iterations=5, taubin_iterations=5):
    print("Smoothing the mesh using Laplacian and Taubin Smoothing...")
    mesh = mesh.filter_smooth_laplacian(number_of_iterations=laplacian_iterations)
    mesh = mesh.filter_smooth_taubin(number_of_iterations=taubin_iterations)
    return mesh

def process_point_cloud(input_path, output_path):
    print(f"Loading point cloud from {input_path}...")
    pcd = o3d.io.read_point_cloud(input_path)
    print(f"Loaded point cloud with {len(pcd.points)} points.")

    pcd = preprocess_pcd(pcd)

    # Step 6: Generate Poisson mesh
    mesh = poisson_mesh(pcd, depth=10, scale=1.5)

    # Step 7: Fill small holes in the mesh
    mesh = fill_mesh_holes(mesh)

    # Step 8: Smooth the mesh
    mesh = smooth_mesh(mesh, laplacian_iterations=5, taubin_iterations=5)

    # Step 9: Save and visualize the final mesh
    o3d.io.write_triangle_mesh(output_path, mesh)
    print("Mesh saved.")

    print("Visualizing the final mesh...")
    o3d.visualization.draw_geometries([mesh], window_name="Refined Mesh")


if __name__ == "__main__":
    input_pcd_file = "data/merge_output/final_cleaned_model.pcd"
    output_mesh_file = "data/mesh_output/mesh_model.ply"

    process_point_cloud(input_pcd_file, output_mesh_file)
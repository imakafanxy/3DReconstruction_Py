import os
import open3d as o3d
import numpy as np
import cv2

def display_inlier_outlier(cloud, ind, title="Inliers and Outliers"):
    """
    Visualize inliers and outliers in the point cloud.
    """
    print(f"Visualizing {title}...")

    # Inliers (points to keep)
    inlier_cloud = cloud.select_by_index(ind)

    if cloud.has_colors():  # If the original point cloud has colors
        inlier_colors = np.asarray(cloud.colors)[ind]  # Preserve original colors for inliers
        inlier_cloud.colors = o3d.utility.Vector3dVector(inlier_colors)
    else:
        # Default color for inliers (gray)
        inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])

    # Outliers (points to remove)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])  # Red color for outliers

    # Visualize inliers and outliers together
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      window_name=title,
                                      width=800,
                                      height=600,
                                      point_show_normal=False)

def remove_specific_color_points(pcd, lower_green=(30, 30, 30), upper_green=(90, 255, 255)):
    """
    Remove points that belong to green-dominant color ranges (e.g., green background).
    """
    print("Removing green-dominant points...")

    # Extract RGB channels and convert to HSV
    colors = np.asarray(pcd.colors)
    colors_uint8 = (colors * 255).astype(np.uint8)  # Convert to 0-255 scale
    hsv_colors = cv2.cvtColor(colors_uint8[np.newaxis, :, :], cv2.COLOR_RGB2HSV)[0]

    # Create a mask for green points
    green_mask = (hsv_colors[:, 0] >= lower_green[0]) & (hsv_colors[:, 0] <= upper_green[0]) & \
                 (hsv_colors[:, 1] >= lower_green[1]) & (hsv_colors[:, 1] <= upper_green[1]) & \
                 (hsv_colors[:, 2] >= lower_green[2]) & (hsv_colors[:, 2] <= upper_green[2])

    # Invert mask to keep non-green points
    inlier_mask = ~green_mask
    print(f"Removed {np.sum(green_mask)} green-dominant points. Remaining points: {np.sum(inlier_mask)}")
    return inlier_mask

def remove_ground_points(pcd, z_threshold):
    """
    Remove ground points below a specified z-axis threshold.
    """
    print(f"Removing ground points below z = {z_threshold}...")
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None

    # Filter points above the z_threshold
    mask = points[:, 2] > z_threshold  # Keep points with z > z_threshold
    filtered_points = points[mask]
    if colors is not None:
        filtered_colors = colors[mask]
    else:
        filtered_colors = None

    # Create a new point cloud with the filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    if filtered_colors is not None:
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    print(f"Ground points removed. Remaining points: {len(filtered_pcd.points)}")
    return filtered_pcd

def remove_statistical_outliers(pcd, nb_neighbors=50, std_ratio=4.0):
    """
    Remove outliers using Statistical Outlier Removal.
    """
    print("Removing statistical outliers...")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    #display_inlier_outlier(pcd, ind, title="After Statistical Outlier Removal")
    print(f"Statistical outliers removed. Remaining points: {len(ind)}")
    return cl

def natural_sort(file_list):
    def natural_key(text):
        import re
        return [int(c) if c.isdigit() else c for c in re.split(r'(\d+)', text)]
    return sorted(file_list, key=natural_key)

def main():
    input_folder = "data/raw"
    output_folder = "data/filtered"
    os.makedirs(output_folder, exist_ok=True)
    '''
    if you makes a new folder, you can change input_folder & output_folder name
    '''

    pcd_files = [f for f in os.listdir(input_folder) if f.startswith("output_") and f.endswith(".pcd")]
    pcd_files = natural_sort(pcd_files)

    for pcd_file in pcd_files:
        input_path = os.path.join(input_folder, pcd_file)
        #o3d.visualization.draw_geometries([o3d.io.read_point_cloud(input_path)], window_name="Input Point Cloud Visualization")

        print(f"\nProcessing file: {input_path}")
        
        # Load the PCD file
        pcd = o3d.io.read_point_cloud(input_path)
        #print(f"Loaded point cloud with {len(pcd.points)} points.")
        
        # Step 2: Remove green-dominant points (color-based filtering)
        inlier_mask_color = remove_specific_color_points(pcd)
        #display_inlier_outlier(pcd, np.where(inlier_mask_color)[0], title="After Removing Green Points")
        '''
        if you want to check inlier/outlier, you can this command(display_inlier_outlier)
        '''
        pcd = pcd.select_by_index(np.where(inlier_mask_color)[0])
        #print(f"Point cloud after color filtering has {len(pcd.points)} points.")

        # Step 3: Remove ground points below z_threshold
        pcd = remove_ground_points(pcd, z_threshold=0)
        #display_inlier_outlier(pcd, np.arange(len(pcd.points)), title="After Removing Ground Points")

        # Step 4: Remove statistical outliers
        pcd = remove_statistical_outliers(pcd, nb_neighbors=50, std_ratio=4.0)

        # Save the processed point cloud
        output_path = os.path.join(output_folder, f"filtered_{pcd_file}")
        o3d.io.write_point_cloud(output_path, pcd)
        #print(f"Processed point cloud saved as: {output_path}")

    print("\nProcessing completed. Check the 'filtered' folder for results.")

if __name__ == "__main__":
    main()

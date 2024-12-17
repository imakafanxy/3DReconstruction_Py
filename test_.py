import os
import open3d as o3d
import numpy as np
import cv2

def natural_sort(file_list):
    def natural_key(text):
        import re
        return [int(c) if c.isdigit() else c for c in re.split(r'(\d+)', text)]
    return sorted(file_list, key=natural_key)

def apply_transformation(pcd, transformation_matrix):
    print("Applying transformation matrix...")
    pcd.transform(transformation_matrix)
    return pcd

def remove_specific_color_points(pcd, lower_green=(30, 30, 30), upper_green=(90, 255, 255)):
    print("Removing green-dominant points...")
    colors = np.asarray(pcd.colors)
    colors_uint8 = (colors * 255).astype(np.uint8)
    hsv_colors = cv2.cvtColor(colors_uint8[np.newaxis, :, :], cv2.COLOR_RGB2HSV)[0]

    green_mask = (hsv_colors[:, 0] >= lower_green[0]) & (hsv_colors[:, 0] <= upper_green[0]) & \
                 (hsv_colors[:, 1] >= lower_green[1]) & (hsv_colors[:, 1] <= upper_green[1]) & \
                 (hsv_colors[:, 2] >= lower_green[2]) & (hsv_colors[:, 2] <= upper_green[2])

    inlier_mask = ~green_mask
    print(f"Removed {np.sum(green_mask)} green-dominant points. Remaining points: {np.sum(inlier_mask)}")
    return pcd.select_by_index(np.where(inlier_mask)[0])

def remove_ground_points(pcd, z_threshold):
    print(f"Removing ground points below z = {z_threshold}...")
    points = np.asarray(pcd.points)
    mask = points[:, 2] > z_threshold
    return pcd.select_by_index(np.where(mask)[0])

def remove_statistical_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    print("Removing statistical outliers...")
    filtered_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=int(nb_neighbors), std_ratio=std_ratio)
    print(f"Statistical outliers removed. Remaining points: {len(filtered_pcd.points)}")
    return filtered_pcd

def preprocess_pcd(pcd, z_threshold,nb_neighbors=50, std_ratio=1.0):
    print("Preprocessing point cloud...")
    pcd = remove_statistical_outliers(pcd, nb_neighbors, std_ratio)
    print(f"Preprocessing complete. Point cloud has {len(pcd.points)} points.")
    return pcd

def rotate_pcd(pcd, index, total_frames):
    rotation_per_frame = 360.0 / total_frames
    rotation_angle = rotation_per_frame * index
    angle_rad = np.radians(rotation_angle)
    rotation_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0],
        [np.sin(angle_rad),  np.cos(angle_rad), 0],
        [0,                 0,                 1]
    ])
    print(f"Rotating point cloud by {rotation_angle:.2f} degrees around z-axis...")
    pcd.rotate(rotation_matrix, center=(0, 0, 0))
    return pcd

def preprocess_and_merge_pcds(directory, z_threshold=0, nb_neighbors=50, std_ratio=1.0):
    pcd_files = [f for f in os.listdir(directory) if f.startswith('filtered') and f.endswith('.pcd')]
    pcd_files = natural_sort(pcd_files)

    if not pcd_files:
        print("No PCD files found for merging.")
        return None

    print(f"Found {len(pcd_files)} PCD files. Starting merge process...")
    total_frames = len(pcd_files)
    voxel_size = 0.005

    merged_pcd = o3d.io.read_point_cloud(os.path.join(directory, pcd_files[0]))
    merged_pcd = merged_pcd.voxel_down_sample(voxel_size = voxel_size)
    merged_pcd = preprocess_pcd(merged_pcd, z_threshold, nb_neighbors, std_ratio)
    merged_pcd = rotate_pcd(merged_pcd, 0, total_frames)

    for i, file_name in enumerate(pcd_files[1:], start=1):
        file_path = os.path.join(directory, file_name)
        try:
            pcd = o3d.io.read_point_cloud(file_path)
            pcd = pcd.voxel_down_sample(voxel_size = voxel_size)
            pcd = preprocess_pcd(pcd, nb_neighbors, std_ratio)
            pcd = rotate_pcd(pcd, i, total_frames)
            merged_pcd += pcd
            print(f"Merged {i + 1}/{total_frames}: {file_name}")
        except Exception as e:
            print(f"[Error] Failed to process {file_name}: {e}")

    return merged_pcd

def final_cleanup(directory):
    final_filtered_file = os.path.join(directory, "final_filtered_model.pcd")
    if not os.path.exists(final_filtered_file):
        print(f"{final_filtered_file} does not exist.")
        return

    print(f"Loading {final_filtered_file} for final outlier removal...")
    final_pcd = o3d.io.read_point_cloud(final_filtered_file)
    final_cleaned_pcd, _ = final_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)

    cleaned_output_file = os.path.join(directory, "final_cleaned_model.pcd")
    o3d.io.write_point_cloud(cleaned_output_file, final_cleaned_pcd)
    print(f"Final cleaned model saved as: {cleaned_output_file}")

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=(0, 0, 0))
    o3d.visualization.draw_geometries([final_cleaned_pcd, coordinate_frame], window_name="Final Cleaned Point Cloud")

def main(directory):
    merged_pcd = preprocess_and_merge_pcds(directory, z_threshold=0, nb_neighbors=50, std_ratio=1.0)

    if merged_pcd is None:
        print("No merged PCD generated.")
        return

    filtered_pcd, _ = merged_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=0.5)
    filtered_output_file = os.path.join(directory, "final_filtered_model.pcd")
    o3d.io.write_point_cloud(filtered_output_file, filtered_pcd)
    print(f"Final filtered model saved as: {filtered_output_file}")

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=(0, 0, 0))
    o3d.visualization.draw_geometries([filtered_pcd, coordinate_frame], window_name="Final Filtered Point Cloud")

    # Final cleanup step
    final_cleanup(directory)

if __name__ == "__main__":
    main("filtered_hw_pose4")

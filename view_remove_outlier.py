import open3d as o3d
import numpy as np

def display_inlier_outlier(cloud, ind):
    """
    Visualize inliers and outliers in the point cloud.
    """
    print("Visualizing inliers and outliers...")

    # 선택된 포인트를 유지하면서 원래 색상을 유지
    inlier_cloud = cloud.select_by_index(ind)

        # 원래 색상을 유지하도록 설정
    if cloud.has_colors():  # 원래 포인트 클라우드에 색상이 있다면
        inlier_colors = np.asarray(cloud.colors)[ind]  # 인덱스에 해당하는 색상 선택
        inlier_cloud.colors = o3d.utility.Vector3dVector(inlier_colors)
    else:
        # 색상이 없으면 기본 색상 설정 (옵션)
        inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # 회색 등

    # Outlier points (points that are removed)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])  # Red for outliers

    # Visualize both inliers and outliers together
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      window_name="Inliers and Outliers",
                                      width=800,
                                      height=600,
                                      point_show_normal=False)

def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Remove outliers using Statistical Outlier Removal.
    """
    print("Removing outliers...")
    pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    display_inlier_outlier(pcd, ind)
    print(f"Outliers removed. Remaining points: {len(pcd_clean.points)}")
    return pcd_clean

def remove_ground_points(pcd, z_threshold):
    """
    Remove ground points below a specified z-axis threshold.
    """
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

def remove_specific_color_points(pcd, red_threshold=0.8, green_threshold=0.8, blue_threshold=0.8,
                                 red_dominant_threshold=0.1, blue_dominant_threshold=0.1):
    """
    Remove points based on specific color conditions:
    - Red-dominant points
    - Blue-dominant points
    
    Parameters:
    - red_threshold (float): Maximum red value for points to remove.
    - green_threshold (float): Maximum green value for points to remove.
    - blue_threshold (float): Maximum blue value for points to remove.
    - red_dominant_threshold (float): Minimum difference between red and other channels to classify as red-dominant.
    - blue_dominant_threshold (float): Minimum difference between blue and other channels to classify as blue-dominant.
    """
    print(f"Removing red-dominant points and blue-dominant points with thresholds...")

    # Extract color channels
    colors = np.asarray(pcd.colors)
    red_channel = colors[:, 0]
    green_channel = colors[:, 1]
    blue_channel = colors[:, 2]

    # Identify red-dominant points
    red_dominant = (red_channel - green_channel > red_dominant_threshold) & \
                   (red_channel - blue_channel > red_dominant_threshold)

    # Identify blue-dominant points
    blue_dominant = (blue_channel - red_channel > blue_dominant_threshold) & \
                    (blue_channel - green_channel > blue_dominant_threshold)

    # Combine conditions to remove both red and blue dominant points
    remove_idx = red_dominant | blue_dominant

    # Filter points
    remaining_points = np.asarray(pcd.points)[~remove_idx]
    remaining_colors = colors[~remove_idx]

    # Create a new point cloud with filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(remaining_points)
    filtered_pcd.colors = o3d.utility.Vector3dVector(remaining_colors)

    print(f"Specific color points removed. Remaining points: {len(filtered_pcd.points)}")
    return filtered_pcd




# Transformation matrix
Mos_gt = np.array([
    [-0.0000, -0.0000,  1.0000,  -0.95],
    [ 0.0000, -1.0000, -0.0000,  1.100],
    [ 1.0000,  0.0000,  0.0000,  1.53],
    [ 0.0000,  0.0000,  0.0000,  1.0000]
])

# Load PCD file
pcd = o3d.io.read_point_cloud("data/raw/output_1.pcd")
print(f"Original point cloud has {len(pcd.points)} points.")
o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")

# Apply transformation
print("Applying transformation...")
#pcd.transform(Mos_gt)

# Remove ground points
z_threshold = 0
pcd_filtered = remove_ground_points(pcd, z_threshold)

# Remove specific color points
pcd_filtered = remove_specific_color_points(pcd_filtered, 
                                            red_threshold=0.8, 
                                            green_threshold=0.8, 
                                            blue_threshold=0.9)


# Remove outliers
pcd_filtered = remove_outliers(pcd_filtered)

# Apply voxel downsampling
voxel_size = 0.005
print(f"Applying voxel downsampling with voxel size {voxel_size}...")
pcd_downsampled = pcd_filtered.voxel_down_sample(voxel_size)
print(f"Downsampled point cloud has {len(pcd_downsampled.points)} points.")

# Save filtered point cloud
output_file = "filtered_person/filter_output2.pcd"
o3d.io.write_point_cloud(output_file, pcd_downsampled)
print(f"Downsampled and filtered point cloud saved as: {output_file}")

# Visualize the final result
print("Displaying transformed, filtered, and downsampled point cloud...")
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd_downsampled, coordinate_frame], window_name="Filtered Point Cloud")

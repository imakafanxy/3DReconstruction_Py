import time
import cv2
import os
import numpy as np
import pyrealsense2 as rs
import open3d as o3d

class RealSenseCameraROI:
    def __init__(self, roi=(220, 40, 720, 580, 0.2, 2.5), display_all=False):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.align = rs.align(rs.stream.color)  # Align depth to color frame
        self.pipeline_profile = self.pipeline.start(self.config)
        self.depth_scale = self.pipeline_profile.get_device().first_depth_sensor().get_depth_scale()
        self.clipping_distance_in_meters = 2.5  # Example distance (adjust as needed)
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale
        self.index = 1
        self.roi = roi
        self.display_all = display_all

        if not os.path.exists("data/raw"):
            os.makedirs("data/raw")
            
    def capture_frame(self, prefix="output"):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            print("Failed to capture depth or color frame.")
            return False

        # Convert depth and color frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Crop based on ROI
        x, y, w, h, d_min, d_max = self.roi
        depth_image = depth_image[y:y + h, x:x + w]
        color_image = color_image[y:y + h, x:x + w]

        # Apply depth filtering based on min/max distance
        depth_image = np.where((depth_image >= d_min / self.depth_scale) &
                            (depth_image <= d_max / self.depth_scale), depth_image, 0)

        # Ensure depth and color frames are aligned
        if depth_image.shape[:2] != color_image.shape[:2]:
            print(f"Shape mismatch: depth={depth_image.shape}, color={color_image.shape}")
            return False

        # Convert BGR to RGB
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Create Open3D RGBD image
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_image),
            o3d.geometry.Image(depth_image),
            depth_scale=1.0 / self.depth_scale,
            convert_rgb_to_intensity=False
        )

        # Create point cloud from RGBD image
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

        # Transform the point cloud
        pcd.transform([
            [-0.0000, -0.0000,  1.0000,  -0.935],
            [ 0.0000, -1.0000, -0.0000,   0.118],
            [ 1.0000,  0.0000,  0.0000,   1.53],
            [ 0.0000,  0.0000,  0.0000,   1.0000]
        ])

        # Save the point cloud
        filename = f"data/raw/{prefix}_{self.index}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.index += 1
        return filename


    def draw_roi_box(self, image):
        x, y, w, h, d_min, d_max = self.roi
        return cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

    def stop(self):
        self.pipeline.stop()

def main():
    camera = RealSenseCameraROI(display_all=True)
    try:
        captured_files = []
        
        while True:
            frames = camera.pipeline.wait_for_frames()
            aligned_frames = camera.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())    
            color_image = camera.draw_roi_box(color_image)

            cv2.imshow('RealSense', color_image)

            key = cv2.waitKey(1)
            
            if key == ord('f'):
                print("Waiting for 10 seconds before starting capture...")
                time.sleep(10)

                start_time = time.time()
                capturing = True
                frame_count = 0

                while capturing:
                    if time.time() - start_time > 34:
                        print("Capture complete.")
                        capturing = False
                        break

                    filename = camera.capture_frame(prefix="output")

                    if not filename:
                        print(f"[ERROR] Capture failed for frame {frame_count + 1}.")
                    else:
                        captured_files.append(filename)
                        print(f"Captured file {frame_count + 1}: {filename}")

                    time.sleep(1)
                    frame_count += 1
                    
            elif key == 27:
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
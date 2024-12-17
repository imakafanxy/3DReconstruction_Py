# **3D Reconstruction with RealSense and Open3D**
## **Project Workflow for 3D Mesh Generation**

---

## **Overview**

This project provides a complete pipeline to capture, filter, merge, and mesh point cloud data to generate 3D models using **Intel RealSense** cameras and the **Open3D** library.

Documentation for 3D Reconstruction with RealSense and Open3D Program

Written by: MyeongJun, Kim in Handong Global University IILAB

Date: Updated 2024-12-17

Program: Python

IDE/Compiler: Visual Studio Code

OS: Win11

---

## **Table of Contents**

1. [Workflow Steps](#workflow-steps)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Code Details](#code-details)
5. [Project Structure](#project-structure)
6. [Examples](#examples)
7. [Future Improvements](#future-improvements)

---

## **Workflow Steps**

The 3D reconstruction pipeline consists of **four key steps**:

1. **Capture**: Record point cloud data from the RealSense camera with ROI applied.
2. **Filter**: Process the point clouds to remove noise, background, and ground.
3. **Merge**: Align and combine multiple point cloud frames.
4. **Mesh**: Convert the final merged point cloud into a 3D mesh.

---

## **Installation**

### **Requirements**

- Python 3.8 - 3.10
- Intel RealSense SDK
- Open3D
- NumPy
- OpenCV

### **Steps to Install**

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/3DReconstruction_Py.git
   cd 3DReconstruction
   ```

2. Install the required Python libraries:
   ```bash
   pip install -r requirements.txt
   ```

3. Install the [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense) and ensure a **USB 3.0** connection for the RealSense camera.

---

## **Usage**

### **Step 1: Capture Point Cloud Data**

1. Run the `capture_main.py` script to record point cloud frames:
   ```bash
   python capture_main.py
   ```

2. **Instructions**:
   - Press `f` to start capturing frames (34 frames are captured with a 10-second delay).
   - Press `ESC` to exit the viewer.

3. The captured point cloud files will be saved in the `data/raw` directory.

---

### **Step 2: Filter the Captured Point Clouds**

1. Run the `go_filter.py` script to clean and filter the raw point cloud data:
   ```bash
   python go_filter.py
   ```

2. **Filter Operations**:
   - Remove green-dominant points (e.g., background removal).
   - Remove ground points based on a z-threshold.
   - Remove statistical outliers.

3. Filtered point clouds will be saved in the `data/filtered` directory.

---

### **Step 3: Merge the Filtered Point Clouds**

1. Run the `merge.py` script to align and merge the filtered point clouds:
   ```bash
   python merge.py
   ```

2. **Process**:
   - The point clouds are rotated and aligned based on the number of frames.
   - Merged point clouds undergo final noise removal.

3. The merged and cleaned point cloud will be saved as `final_cleaned_model.pcd` in the `data/merge_output` directory.

---

### **Step 4: Generate the 3D Mesh**

1. Run the `mesh.py` script to generate the 3D mesh:
   ```bash
   python mesh.py
   ```

2. **Mesh Process**:
   - Perform Poisson surface reconstruction.
   - Fill small holes in the mesh.
   - Apply smoothing operations (Laplacian and Taubin smoothing).

3. The final mesh will be saved as `mesh_model.ply` in the `data/mesh_output` directory.

4. The mesh will be visualized after generation.

---

## **Code Details**

### **1. capture_main.py**

- **Purpose**: Captures point cloud frames from a RealSense camera.
- **Key Features**:
  - ROI cropping.
  - Depth filtering based on min and max distances.
- **Output**: Saves `.pcd` files to `data/raw`.

### **2. go_filter.py**

- **Purpose**: Filters the captured point cloud data.
- **Filtering Steps**:
  1. Remove green points (background).
  2. Remove ground points below a threshold.
  3. Apply statistical outlier removal.
- **Output**: Saves cleaned `.pcd` files to `data/filtered`.

### **3. merge.py**

- **Purpose**: Aligns and merges multiple filtered point clouds.
- **Process**:
  - Rotates point clouds for alignment.
  - Removes additional noise.
- **Output**: Saves merged `.pcd` file to `data/merge_output`.

### **4. mesh.py**

- **Purpose**: Generates a 3D mesh from the merged point cloud.
- **Steps**:
  - Poisson reconstruction.
  - Hole filling.
  - Mesh smoothing.
- **Output**: Saves the 3D mesh as `.ply` to `data/mesh_output`.

---

## **Project Structure**

```plaintext
3DReconstruction/
│
├── data/
│   ├── raw/                # Raw point cloud data
│   ├── filtered/           # Filtered point clouds
│   ├── merge_output/       # Merged and cleaned point cloud files
│   └── mesh_output/        # Final 3D mesh files
│
├── src/
│   ├── capture_main.py     # Capture point clouds
│   ├── go_filter.py        # Filter point clouds
│   ├── merge.py            # Merge point clouds
│   └── mesh.py             # Generate 3D mesh
│
├── README.md               # Project documentation
├── requirements.txt        # Python dependencies
└── main.py                 # (Optional) Main pipeline script
```

---

## **Example Workflow**

1. **Capture Frames**:
   ```bash
   python capture_main.py
   ```

2. **Filter Point Clouds**:
   ```bash
   python go_filter.py
   ```

3. **Merge Filtered Files**:
   ```bash
   python merge.py
   ```

4. **Generate 3D Mesh**:
   ```bash
   python mesh.py
   ```

---

## **Future Improvements**

- Improve filtering accuracy with machine learning techniques.
- Add real-time 3D reconstruction visualization.
- Optimize Poisson reconstruction for faster mesh generation.
- Support multi-camera setups for more detailed scans.

---

## **License**

This project is licensed under the MIT License.


import open3d as o3d
import numpy as np


def calculate_boundary_length(mesh):
    """
    Calculate the length of boundary edges in the mesh.
    """
    print("Extracting boundary edges...")
    triangles = np.asarray(mesh.triangles)
    edges = {}

    # Count the occurrence of each edge
    for triangle in triangles:
        for i in range(3):
            edge = tuple(sorted((triangle[i], triangle[(i + 1) % 3])))
            if edge in edges:
                edges[edge] += 1
            else:
                edges[edge] = 1

    # Boundary edges occur only once
    boundary_edges = [edge for edge, count in edges.items() if count == 1]

    # Calculate boundary length
    vertices = np.asarray(mesh.vertices)
    boundary_length = 0.0
    for edge in boundary_edges:
        v1, v2 = vertices[edge[0]], vertices[edge[1]]
        boundary_length += np.linalg.norm(v1 - v2)

    print(f"Boundary length: {boundary_length:.2f}")
    return boundary_length


def slice_and_calculate_length(mesh, plane_origin, plane_normal):
    """
    Slice the mesh using a plane and calculate the length of the resulting contour.
    """
    print("Slicing the mesh with a plane...")
    slice = mesh.section(plane_origin=plane_origin, plane_normal=plane_normal)
    print("Section created.")

    # Calculate the length of the section (sum of edge lengths)
    section_length = 0.0
    for line in slice.lines:
        p1, p2 = np.asarray(slice.points)[line[0]], np.asarray(slice.points)[line[1]]
        section_length += np.linalg.norm(p1 - p2)

    print(f"Section length: {section_length:.2f}")
    return section_length


def main():
    input_mesh_file = "mesh/poisson_surface_mesh.ply"  # Replace with your mesh file

    # Load the mesh
    print(f"Loading mesh from {input_mesh_file}...")
    mesh = o3d.io.read_triangle_mesh(input_mesh_file)
    print("Mesh loaded.")

    # Calculate the boundary length
    print("Calculating the boundary length...")
    boundary_length = calculate_boundary_length(mesh)
    print(f"Boundary length: {boundary_length:.2f}")

    # Slice the mesh and calculate contour length
    print("Calculating the contour length from a slicing plane...")
    plane_origin = [0, 0, 0]  # Plane origin
    plane_normal = [0, 0, 1]  # Plane normal
    slice_length = slice_and_calculate_length(mesh, plane_origin, plane_normal)
    print(f"Slice contour length: {slice_length:.2f}")

    # Visualize the mesh
    print("Visualizing the mesh...")
    o3d.visualization.draw_geometries([mesh], window_name="Mesh Visualization")


if __name__ == "__main__":
    main()

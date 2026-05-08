from scipy.spatial import KDTree
import open3d as o3d
import numpy as np
from sklearn.metrics.pairwise import pairwise_distances


def filter_points(points_array, threshold_distance, nr_neighbors=3):
    """Takes in array of points and does a median filter using KDtree"""
    # Construct a KDTree for efficient nearest neighbor search
    tree = KDTree(points_array)

    # Query the tree to find the indices of neighboring points within the threshold distance
    filtered_indices = []
    for i in range(len(points_array)):
        indices = tree.query_ball_point(points_array[i], threshold_distance)
        if len(indices) > nr_neighbors:  # At least one neighbor (excluding itself)
            filtered_indices.append(i)

    # Extract the filtered points
    filtered_points = points_array[filtered_indices]

    return filtered_points


def preprocess_point_cloud(point_cloud):
    # Convert numpy array to Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)

    # Downsample the point cloud to reduce noise
    downsampled_pcd = pcd.voxel_down_sample(
        voxel_size=0.01)  # Adjust voxel size as needed

    # Estimate normals for the downsampled point cloud
    downsampled_pcd.estimate_normals()

    # Remove outlier points based on local geometry
    inlier_cloud, _ = downsampled_pcd.remove_statistical_outlier(
        50, 1)  # Adjust parameters as needed

    # Convert Open3D PointCloud object back to numpy array
    filtered_point_cloud = np.asarray(inlier_cloud.points)

    return filtered_point_cloud


def extract_points_on_lines(point_cloud, min_distance=1, min_line_length=1.0, distance_to_line=0.1):
    """min_distance = minimum distance between points in a line segment.
    min_line_length = minimum length of a line segment.
    distance_to_line = minimum distance between a point and a line segment."""
    # Initialize variables to store line segments
    line_segments = []
    current_line = []

    # Segment the point cloud into line segments
    for i, point in enumerate(point_cloud):
        if current_line:
            dist_to_line = np.linalg.norm(point[:2] - current_line[-1][:2])
            if dist_to_line < min_distance:
                current_line.append(point)
            else:
                if len(current_line) > 1:
                    line_segments.append(current_line)
                current_line = []
        else:
            current_line.append(point)

    # Add the last line segment if it meets the minimum length requirement
    if len(current_line) > 1:
        line_segments.append(current_line)

    # Filter out line segments shorter than min_line_length
    line_segments = [segment for segment in line_segments if np.linalg.norm(
        segment[-1][:2] - segment[0][:2]) >= min_line_length]

    # Extract points closer than distance_to_line to any line segment
    close_points = []
    for point in point_cloud:
        for segment in line_segments:
            dist_to_line = np.linalg.norm(np.cross(
                segment[-1][:2] - segment[0][:2], segment[0][:2] - point[:2])) / np.linalg.norm(segment[-1][:2] - segment[0][:2])
            if dist_to_line < distance_to_line:
                close_points.append(point)
                break

    return line_segments, np.asarray(close_points)


def smooth_point_cloud(point_cloud, radius):
    smoothed_cloud = []
    tree = KDTree(point_cloud[:, :2])  # Create KDTree for 2D points

    for point in point_cloud:
        # Query the KDTree for points within the radius
        neighbors_indices = tree.query_ball_point(point[:2], radius)
        if neighbors_indices:
            # Calculate the mean of the neighbors' coordinates
            mean_x = np.mean(point_cloud[neighbors_indices][:, 0])
            mean_y = np.mean(point_cloud[neighbors_indices][:, 1])
            mean_z = np.mean(point_cloud[neighbors_indices][:, 2])
            smoothed_cloud.append([mean_x, mean_y, mean_z])
        else:
            # If no neighbors found within the radius, keep the original point
            smoothed_cloud.append(point)

    return np.array(smoothed_cloud)

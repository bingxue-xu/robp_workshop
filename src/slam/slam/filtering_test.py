import numpy as np
import open3d as o3d

from process_cloud import *

# TO RUN THIS SCRIPT YOU HAVE TO REMOVE THE DOTS IN FRONT OF IMPORTS IN SCAN_MATCHING.PY
for i in range(165):
    string = 'slam/slam/Data/spin%d.txt' % i
    array = np.loadtxt(string)
    # FOR INCOMING CLOUDS
    # _, array = extract_points_on_lines(
    #   array, 0.5, 0.1, 0.5)
    # array = filter_points(array, 0.5, 5)
    ###########
    # array = filter_points(array, 0.5, 150)
    # array = filter_points(array, 1, 350)
    # array = preprocess_point_cloud(array)
    # _, array = extract_points_on_lines(
    #    array, 0.8, 0.5, 0.5)
    # array = filter_points(array, 1, 150)
    # _, array = extract_points_on_lines(
    #   array, 0.5, 0.5, 0.5)
    # array = smooth_point_cloud(array, 0.1)
    print(len(array))

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(
        array)
    o3d.visualization.draw_geometries([point_cloud],
                                      zoom=-0.5,
                                      front=[0.2, 0.2125, 0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

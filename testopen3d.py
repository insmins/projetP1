import open3d as o3d
import numpy as np

# load point cloud
points = np.loadtxt("uncube_cam_0.txt")

pcl = o3d.geometry.PointCloud()
pcl.points = o3d.utility.Vector3dVector(points)

# find plane
pt_to_plane_dist = 0.02
print("yo")
plane_model, inliers = pcl.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=10)
print("cc")
[a, b, c, d] = plane_model

inlier_cloud = pcl.select_by_index(inliers)
outlier_cloud = pcl.select_by_index(inliers, invert=True)

o3d.visualization.draw_geometries([outlier_cloud])

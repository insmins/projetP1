import open3d as o3d
import numpy as np

# load point cloud
points = np.loadtxt("maxipoints.txt")

# supp en dessous de la table

xmin = -0.54
xmax = -0.09
ymin = -0.03
ymax = 0.27
zmin = 0.014
asupp = []
for i, point in enumerate(points):
    if point[0] <= xmin or point[0] >= xmax or point[1] <= ymin or point[1] >= ymax or point[2] <= zmin:
        asupp.append(i)


points = np.delete(points, asupp, axis = 0)

# creation du pointcloud open3d
pcl = o3d.geometry.PointCloud()
pcl.points = o3d.utility.Vector3dVector(points)

# # data pre processing
# pcl_center = pcl.get_center()
# pcl.translate(-pcl_center)

# supprimer les points aberrants
nn = 16
std_multiplier = 10
filtered_pcl = pcl.remove_statistical_outlier(nn, std_multiplier)
outliers = pcl.select_by_index(filtered_pcl[1], invert=True)
filtered_pcl = filtered_pcl[0]

# voxel downsampling
voxel_size = 0.001
pcl_downsampled = filtered_pcl.voxel_down_sample(voxel_size=voxel_size)

# find plane
pt_to_plane_dist = 0.01
plane_model, inliers = pcl_downsampled.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model

inlier_cloud = pcl_downsampled.select_by_index(inliers)
outlier_cloud = pcl_downsampled.select_by_index(inliers, invert=True)

box = outlier_cloud.get_oriented_bounding_box()

o3d.visualization.draw_geometries([outlier_cloud, box])

# # test nouveau plan
# pcl = outlier_cloud.remove_statistical_outlier(nn, std_multiplier)[0]
# o3d.visualization.draw_geometries([pcl])
# plane_model, inliers = pcl.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
# [a, b, c, d] = plane_model

# inlier_cloud = pcl.select_by_index(inliers)
# outlier_cloud = pcl.select_by_index(inliers, invert=True)

# o3d.visualization.draw_geometries([inlier_cloud])

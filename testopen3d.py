import open3d as o3d
import numpy as np
from test_cube import tourner, translater, cube, centre, translater_cube, tourner_cube
import polyscope as ps

Centre = centre[0]

# fonction
def ransac_cube(points, num_iterations=1000, threshold_max=0.06, threshold_min=0.0325):
    best_params = None
    best_distance = -1
    best_centre = None

    for _ in range(num_iterations):
        sample = points[np.random.randint(0, points.shape[0])]
        alpha = np.random.uniform(0, 2*np.pi)
        beta = np.random.uniform(0, 2*np.pi)

        centre = translater(sample, tourner(alpha, beta, Centre))

        distances = 0
        n = 0
        for p in points:
            dist = np.sqrt((p[0]-centre[0])**2 + (p[1]-centre[1])**2 + (p[2]-centre[2])**2)
            # min_dist = 100
            # for c in nouveau_cube:
            #     dist = np.sqrt((p[0]-c[0])**2 + (p[1]-c[1])**2 + (p[2]-c[2])**2)
            #     if dist < min_dist:
            #         min_dist = dist
            if dist < threshold_max and dist > threshold_min:
                distances += dist
                n += 1
        
        distances = distances/n
        if distances < best_distance or best_distance == -1:
            best_distance = distances
            best_params = [sample, alpha, beta]
            best_centre = centre
    return best_params, best_distance, best_centre
            


# load point cloud
points = np.loadtxt("maxipoints.txt")

# Filtrage des points dans la zone de travail
xmin = -0.54
xmax = -0.09
ymin = -0.03
ymax = 0.27
zmin = 0.030 # Avec table enlevée
# zmin = 0.014 # Avec table incluse

points = np.array([p for p in points if p[0] > xmin and p[0] < xmax and p[1] > ymin and p[1] < ymax and p[2] > zmin])

# asupp = []
# for i, point in enumerate(points):
#     if point[0] <= xmin or point[0] >= xmax or point[1] <= ymin or point[1] >= ymax or point[2] <= zmin:
#         asupp.append(i)


# points = np.delete(points, asupp, axis = 0)

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
voxel_size = 0.003
pcl_downsampled = filtered_pcl.voxel_down_sample(voxel_size=voxel_size)

points = np.asanyarray(pcl_downsampled.points)

best_cube_params, _, CENTER = ransac_cube(points, num_iterations=5000)

best_cube = translater_cube(best_cube_params[0], tourner_cube(best_cube_params[1], best_cube_params[2], cube))


ps.init()
ps.register_point_cloud("my points", points)
ps.register_point_cloud("centre", np.array([CENTER]))
ps.register_point_cloud("cube", best_cube)
ps.show()

# # find plane
# pt_to_plane_dist = 0.01
# plane_model, inliers = pcl_downsampled.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
# [a, b, c, d] = plane_model

# inlier_cloud = pcl_downsampled.select_by_index(inliers)
# outlier_cloud = pcl_downsampled.select_by_index(inliers, invert=True)

# o3d.visualization.draw_geometries([outlier_cloud])

# box = outlier_cloud.get_oriented_bounding_box()

# o3d.visualization.draw_geometries([outlier_cloud, box])

# # test nouveau plan
# pcl = outlier_cloud.remove_statistical_outlier(nn, std_multiplier)[0]
# o3d.visualization.draw_geometries([pcl])
# plane_model, inliers = pcl.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
# [a, b, c, d] = plane_model

# inlier_cloud = pcl.select_by_index(inliers)
# outlier_cloud = pcl.select_by_index(inliers, invert=True)

# o3d.visualization.draw_geometries([inlier_cloud])

import open3d as o3d
import numpy as np
from test_cube import tourner, translater, cube, centre, translater_cube, tourner_cube
import polyscope as ps
from testgramschmit import gramschmit

# centre du cube de base
Centre = centre[0]

# fonction
def ransac_cube(points, num_iterations=1000, threshold_max=0.058, threshold_min=0.0325):
    """
    Calcule le meilleur cube parmi num_iterations de cubes aléatoires
    """
    # contiennent le meilleur match
    best_params = None
    best_number = -1
    best_centre = None
    best_inliers = None

    for _ in range(num_iterations):
        # un cube = un point, un alpha, un beta
        sample = points[np.random.randint(0, points.shape[0])]
        alpha = np.random.uniform(0, 2*np.pi)
        beta = np.random.uniform(0, 2*np.pi)

        # calcul de la position du centre du cube
        centre = translater(sample, tourner(alpha, beta, Centre))

        n = 0
        inliers = []
        for i, p in enumerate(points):
            # calcul de la distance entre chaque point et le centre du cube
            dist = np.sqrt((p[0]-centre[0])**2 + (p[1]-centre[1])**2 + (p[2]-centre[2])**2)

            # threshold max et min = on regarde si le point est entre le point le plus éloigné
            # et le moins éloigné du centre (entre le centre d'une face ~ 0.0325 et un coin du cube ~ 0.06)
            if dist < threshold_max and dist > threshold_min:
                inliers.append(i)
                n += 1
        
        if n > best_number or best_number == -1:
            best_number = n
            best_params = [sample, alpha, beta]
            best_centre = centre
            best_inliers = inliers
    return best_params, best_inliers, best_centre
            


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

# normal calculation
pcl_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))


# RANSAC
best_cube_params, inliers, CENTER = ransac_cube(points, num_iterations=5000)

# Angle matching
pcl_inliers = pcl_downsampled.select_by_index(inliers)

# premier plan
plane_model, inliersplane = pcl_inliers.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
inliersplane_cloud = pcl_inliers.select_by_index(inliersplane)
outliersplane_cloud = pcl_inliers.select_by_index(inliersplane, invert=True)

normales_1 = np.asarray(inliersplane_cloud.normals)

normales_1 = [np.sign(np.dot(normales_1[i], normales_1[0])) * normales_1[i]  for i in range(normales_1.shape[0])]
inliersplane_cloud.normals = o3d.utility.Vector3dVector(np.array(normales_1))
moy_norm_1 = np.mean(inliersplane_cloud.normals, axis=0)
moy_norm_1 = moy_norm_1 / np.linalg.norm(moy_norm_1)
print(moy_norm_1)

pcl_center = o3d.geometry.PointCloud()
pcl_center.points = o3d.utility.Vector3dVector(np.array([CENTER]))
pcl_center.normals = o3d.utility.Vector3dVector(np.array([moy_norm_1]))

o3d.visualization.draw_geometries([inliersplane_cloud, pcl_center])

# deuxieme plan
plane_model, inliersplane = outliersplane_cloud.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)
inliersplane_cloud = outliersplane_cloud.select_by_index(inliersplane)
outliersplane_cloud = outliersplane_cloud.select_by_index(inliersplane, invert=True)

normales_2 = np.asarray(inliersplane_cloud.normals)

normales_2 = [np.sign(np.dot(normales_2[i], normales_2[0])) * normales_2[i]  for i in range(normales_2.shape[0])]
inliersplane_cloud.normals = o3d.utility.Vector3dVector(np.array(normales_2))
moy_norm_2 = np.mean(inliersplane_cloud.normals, axis=0)
moy_norm_2 = moy_norm_2 / np.linalg.norm(moy_norm_2)
print(moy_norm_2)

pcl_center.normals = o3d.utility.Vector3dVector(np.array([moy_norm_2]))

o3d.visualization.draw_geometries([inliersplane_cloud, pcl_center])

angle = np.acos(np.dot(moy_norm_1, moy_norm_2))
if angle > 1.57-0.52 and angle < 1.57+0.52:
    print("oui")
else:
    print("non")


# troisieme plan
plane_model, inliersplane = outliersplane_cloud.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)
inliersplane_cloud = outliersplane_cloud.select_by_index(inliersplane)
outliersplane_cloud = outliersplane_cloud.select_by_index(inliersplane, invert=True)

normales_3 = np.asarray(inliersplane_cloud.normals)

normales_3 = [np.sign(np.dot(normales_3[i], normales_3[0])) * normales_3[i]  for i in range(normales_3.shape[0])]
inliersplane_cloud.normals = o3d.utility.Vector3dVector(np.array(normales_3))
moy_norm_3 = np.mean(inliersplane_cloud.normals, axis=0)
moy_norm_3 = moy_norm_3 / np.linalg.norm(moy_norm_3)
print(moy_norm_3)

pcl_center.normals = o3d.utility.Vector3dVector(np.array([moy_norm_3]))

o3d.visualization.draw_geometries([inliersplane_cloud, pcl_center])

angle = np.acos(np.dot(moy_norm_1, moy_norm_3))
angle2 = np.acos(np.dot(moy_norm_2, moy_norm_3))
if angle > 1.57-0.52 and angle < 1.57+0.52 and angle2 > 1.57-0.52 and angle2 < 1.57+0.52:
    print("oui")
else:
    print("non")



u1, u2, u3 = gramschmit(moy_norm_1, moy_norm_2, moy_norm_3)

pcl_center.points = o3d.utility.Vector3dVector(np.array([CENTER, CENTER, CENTER]))
pcl_center.normals = o3d.utility.Vector3dVector(np.array([u1, u2, u3]))

o3d.visualization.draw_geometries([pcl_inliers, pcl_center])

VECTEUR = [0.0, 0, 0]
for u in [u1, u2, u3]:
    if np.abs(np.dot(u, [0.0, 0, 1])) > np.abs(np.dot(VECTEUR, [0.0, 0, 1])):
        VECTEUR = u

# A RETURN : VECTEUR ET CENTRE
print(VECTEUR, CENTER)



# best_center_normal = None
# center_normal = [np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)]
# mul = 1
# normale = pcl_inliers.normals[np.random.randint(0, len(inliers))]
# print(f'{normale=}')
# old_angle = np.dot(center_normal, normale)
# for _ in range(400):
#     center_normal = np.random.rand(3)
#     # center_normal = center_normal / np.linalg.norm(center_normal)
#     angle = np.dot(center_normal, normale)
#     if angle < old_angle:
#         old_angle = angle
#         best_center_normal = center_normal

# pcl_center = o3d.geometry.PointCloud()
# pcl_center.points = o3d.utility.Vector3dVector(np.array([CENTER]))
# pcl_center.normals = o3d.utility.Vector3dVector(np.array([best_center_normal]))

# o3d.visualization.draw_geometries([pcl_inliers, pcl_center])



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

import polyscope as ps
from Camera import Camera
import numpy as np
import cv2
from filter_outliers import up_down_limits

ps.init()

# recup données caméra
cam = Camera()
frames, aligned_frames, aligned_depth_frame, color_frame = cam.updateCam()

depth_image = np.asanyarray(aligned_depth_frame.get_data())

# retirer les valeurs aberrantes
flat_depth = depth_image.flatten()
# flat_depth = removeOutliers(flat_depth, 2)
borne_inf, borne_sup = up_down_limits(flat_depth, 2)
mediane = np.median(flat_depth)

# valeurs aberrantes = mediane et creation de la liste des points x, y, z
xyz = []
for x in range(depth_image.shape[0]):
    for y in range(depth_image.shape[1]):
        if depth_image[x, y] <=borne_inf or depth_image[x, y]>= borne_sup: #si en dehors des bornes alors valeur aberrante
            depth_image[x, y] = mediane
        xyz.append([x, y, depth_image[x, y]])        

xyz = np.asanyarray(xyz)

np.savetxt("xyz_traite.txt", xyz)



# visualize!
ps_cloud = ps.register_point_cloud("my points", xyz)
ps.show()

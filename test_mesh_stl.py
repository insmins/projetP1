import numpy as np
from stl import mesh
import matplotlib.tri as tri
import Camera
from filter_outliers import up_down_limits
import datetime

# recup données caméra
cam = Camera.Camera()
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
            depth_image[x][y] = mediane
        xyz.append([x, y, depth_image[x, y]])  

xyz = np.asanyarray(xyz)      

# # chargement de xyz depuis fichier exemple pour test sans cam
# xyz = np.loadtxt("xyz_traite.txt")

# creation de la liste de tous les x tous les y
touslesx = xyz[:, 0]
touslesy = xyz[:, 1]
touslesz = xyz[:, 2]

# creation des triangles
triangles = tri.Triangulation(touslesx, touslesy)

# Create the mesh
cube = mesh.Mesh(np.zeros(triangles.triangles.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(triangles.triangles):
    for j in range(3):
        cube.vectors[i][j] = xyz[f[j],:]

# Write the mesh to file "cube_MMddhhmm.stl" in the stl_file folder
t = datetime.datetime.now()
cube.save('stl_file/cam_'+t.strftime('%m%d%H%M')+'.stl')

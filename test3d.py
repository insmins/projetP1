import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import pymeshlab as pym
import Camera
from filter_outliers import up_down_limits

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

# creation de la liste de tous les x tous les y
touslesx = xyz[:, 0]
touslesy = xyz[:, 1]
touslesz = xyz[:, 2]

# creation des triangles
triangles = tri.Triangulation(touslesx, touslesy)

# # creation du mesh
# mesh = pym.Mesh(xyz, triangles.triangles)

# # create a new MeshSet
# ms = pym.MeshSet()

# # add the mesh to the MeshSet
# ms.add_mesh(mesh, "table_mesh")

# # save the current mesh
# ms.save_current_mesh("C:/Users/projetP1/Downloads/saved_cube_from_array.ply")

# affichage (pas opti) des triangles sur matplotlib
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.plot_trisurf(touslesx, touslesy, touslesz, linewidth=0.2, antialiased=True, triangles=triangles.triangles)

plt.show()
print("fini")
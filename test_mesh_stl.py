import numpy as np
from stl import mesh
import matplotlib.tri as tri
import Camera
from filter_outliers import up_down_limits
import datetime

# recup données caméra
cam = Camera.Camera()
cam.updateCam() 
# creation liste xyz
xyz=cam.create_xyz()
# creation des triangles
triangles =cam.create_xyz()

# Create the mesh
cube = mesh.Mesh(np.zeros(triangles.triangles.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(triangles.triangles):
    for j in range(3):
        cube.vectors[i][j] = xyz[f[j],:]

# Write the mesh to file "cube_MMddhhmm.stl" in the stl_file folder
t = datetime.datetime.now()
cube.save('stl_file/cam_'+t.strftime('%m%d%H%M')+'.stl')

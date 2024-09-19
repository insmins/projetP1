import polyscope as ps
from Camera import Camera
import numpy as np
import cv2
from filter_outliers import up_down_limits

ps.init()

# # recup données caméra
# cam = Camera()
# cam.updateCam() 
# xyz=cam.create_xyz()

xyz = np.loadtxt("cam_1.txt")


# np.savetxt("xyz_traite.txt", xyz)

# visualize!
ps_cloud = ps.register_point_cloud("my points", xyz)
ps.show()

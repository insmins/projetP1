import numpy as np
from Robot import Robot
import polyscope as ps

# CREER LA MAXI LISTE de points (prend du temps)
Points = [np.loadtxt("uncube_cam_0.txt"), np.loadtxt("uncube_cam_1.txt"), np.loadtxt("uncube_cam_2.txt"), np.loadtxt("uncube_cam_3.txt")]
robot = Robot()
poscam = [robot.pos_cam_1, robot.pos_cam_2, robot.pos_cam_3, robot.pos_cam_4]

maxi_points = []
for i, points in enumerate(Points):
    for point in points :
        maxi_points.append(robot.cam2base(point.tolist(), poscam[i]))

maxi_points = np.asanyarray(maxi_points)

np.savetxt("maxipoints.txt", maxi_points)

# # load la maxi liste au lieu de la créer
# maxi_points = np.loadtxt("maxipoints.txt")


# enlever les points plus bas que le plateau
asupp = []
for i, point in enumerate(maxi_points):
    if point[2] < 0.014: # 0.014 est une valeur juste en dessous de la table
        asupp.append(i)

maxi_points = np.delete(maxi_points, asupp, axis = 0)


# visualize!
ps.init()
ps_cloud = ps.register_point_cloud("my points", maxi_points)
ps.show()

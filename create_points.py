from Robot import Robot
from Camera import Camera
import numpy as np

robot = Robot()
cam = Camera()

# init
robot.bouger(robot.pos_init)

# positions de prise de vue
pos_prise = [robot.pos_cam_1, robot.pos_cam_2, robot.pos_cam_3, robot.pos_cam_4]

for i in range(4):
    robot.bouger(pos_prise[i])
    cam.updateCam()
    # depth_image = np.asanyarray(cam.aligned_depth_frame.get_data())

    xyz = cam.create_xyz()
    pos = cam.positions_xyz(xyz)
    np.savetxt(f"cam_{i}.txt", pos)

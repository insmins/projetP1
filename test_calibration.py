from Robot import Robot
from Camera import Camera
import cv2
import numpy as np

# recuperer points camera
cam = Camera()
cam.updateCam() 
robot = Robot()
robot.connexion()

#crea mask rouge
frame = np.asanyarray(cam.color_frame.get_data())
# conversion hsv
frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# creation des masques
rouge = cam.mask_rouge(frame_hsv)


# centre
centre = cam.centre(rouge)
cv2.circle(frame, (int(centre[0]), int(centre[1])), int(1), [0, 0, 255], 2)
# cv2.imshow("centre", frame)
print(centre[0], centre[1])
# position XYZ
posXYZCam = cam.positionXYZ(centre)
print(posXYZCam)
posXYZBase = robot.cam2base(posXYZCam)

posdessus=robot.calcul_pos_relative(dz=0.025, pos=np.transpose(posXYZBase).tolist() + [0, -np.pi, 0])

while(1):
    cv2.imshow("centre",frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
            break

robot.robot_c.moveL(posdessus, 0.5, 0.3)
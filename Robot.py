"""
Classe avec fonctions utiles pour le robot
"""
# import
import rtde_receive
import rtde_control
from Transfo import create_matrice
import numpy as np

class Robot :
    def __init__(self):
        # init communication robot
        self.robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
        self.robot_c = rtde_control.RTDEControlInterface("10.2.30.60")

    def bouger_relatif(self, dx=0, dy=0, dz=0):
        pos = self.robot_r.getActualTCPPose()
        pos = [pos[0]+dx,pos[1]+dy,pos[2]+dz,pos[3],pos[4],pos[5]]
        return pos
    
    def cam2base(self, objetCam):
        objetCam = np.transpose(objetCam + [1])

        T_cam2gripper = np.load("../FinalTransforms/T_cam2gripper_Method_0.npz")['arr_0']
        print(T_cam2gripper )
        # T_cam2gripper[0, 3] = 0.100
        T_cam2gripper[1, 3] = 0.023
        T_cam2gripper[2, 3] = -0.210
        posePrise = self.robot_r.getActualTCPPose()
        T_gripper2base = create_matrice(posePrise)

        res = T_gripper2base @ T_cam2gripper @ objetCam
        print(f'{res=}')
        return res[:3]


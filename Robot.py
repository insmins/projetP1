"""
Classe avec fonctions utiles pour le robot
"""
# import
import rtde_receive
import rtde_control
from Transfo import create_matrice
import numpy as np
from Pince import Pince
import time

class Robot :
    def __init__(self):
        # variable du robot
        self.num_cube = 0
        self.pos_init =[-0.27961,-0.11156,0.23741,0.135,-3.128,0.144] 
        self.pos_depot_cube = [-0.48118,-0.26843,0.06306,0.082,-3.120,0.114]
        self.pos_cam_1 = [-0.22973,0.06416,0.29767,0.059,-3.210,0.160] 
        self.pos_cam_2 = [-0.37208,0.12440,0.28392,0.135,-3.672,-0.017] 
        self.pos_cam_3 = [-0.36526,-0.02440,0.31308,2.016,-3.311,-0.570] 
        self.pos_cam_4 = [-0.17128,0.00558,0.24855,1.211,3.292,0.218]
        self.delta_x = 0.083 #(en mm) decalage en x pour la pose des cubes 
        self.delta_y = 0.083 #(en mm) decalage en y pour la pose des cubes 

        #move to pose initiale
        # self.robot_c.moveL(self.pos_init, 0.5, 0.3)


    def connexion(self):
        self.robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
        self.robot_c = rtde_control.RTDEControlInterface("10.2.30.60")

    def calcul_pos_relative(self, dx=0, dy=0, dz=0, pos = None):
        if pos is None :
            pos = self.robot_r.getActualTCPPose()
        pos = [pos[0]+dx,pos[1]+dy,pos[2]+dz,pos[3],pos[4],pos[5]]
        return pos
    
    def cam2base(self, objetCam):
        objetCam = np.transpose(objetCam + [1])

        T_cam2gripper = [[ 0.04853044,  0.99880257,  0.00618264,  0.10201555],
                        [-0.99542155,  0.047854,    0.08274014,  0.0217057 ],
                        [ 0.0823452,  -0.01016975,  0.99655198, -0.150],
                        [ 0.  ,        0.    ,      0.   ,       1.        ]]
        # T_cam2gripper[0, 3] = 0.100
        # T_cam2gripper[1, 3] = 0.023
        # T_cam2gripper[2, 3] = -0.210
        posePrise = self.robot_r.getActualTCPPose()
        T_gripper2base = create_matrice(posePrise)

        res = T_gripper2base @ T_cam2gripper @ objetCam
        # print(f'{res=}')
        return res[:3]

    def deconnexion(self): 
        self.robot_c.disconnect()

    def bouger(self, pos, speed=0.5, acceleration=0.3):
        self.connexion()
        self.robot_c.moveL(pos, speed, acceleration)
        self.deconnexion()

    def rangement(self, pince: Pince):
        #calcul pos_rangement en fonction de self.num_cube
        pos_rangement= self.calcul_pos_relative(self.delta_x * (self.num_cube//3), self.delta_y* (self.num_cube%3), pos=self.pos_depot_cube)
        print(f"{pos_rangement=}")

        #bouger à pos_rangement (avec pos_intermediaire au dessus)
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement)) #verif si z + ou -
        self.bouger(pos_rangement, 0.5, 0.3)  
        
        #lacher
        pince.lacher()
        
        #remonter
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement)) #verif si z + ou -

        #maj compteur cube
        self.num_cube +=1
        print(self.num_cube)


if __name__ == "__main__":
    robot = Robot()
    pince = Pince()
    robot.rangement(pince)
    robot.bouger(robot.pos_init)
    # time.sleep(3)
    # pince.prise()
    # time.sleep(2)
    robot.rangement(pince)

    robot.bouger(robot.pos_cam_1)
    robot.bouger(robot.pos_cam_2)
    robot.bouger(robot.pos_cam_3)
    robot.bouger(robot.pos_cam_4)

    robot.robot_c.stopScript()

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
        self.pos_cam_2 = [-0.35594,0.16063,0.27617,0.097,2.538,-0.192] 
        self.pos_cam_3 = [-0.14889,-0.02927,0.20331,0.196,3.455,0.276] 
        self.pos_cam_4 = [-0.17349,0.15271,0.24970,1.610,2.644,-0.680]
        self.pos_cam_5 = [-0.21852,0.08733,0.25996,2.090,2.747,-0.771]
        self.pos_cam_6 = [-0.32933,-0.00430,0.26788,2.582,-2.630,-1.215]
        self.delta_x = 0.083 #(en mm) decalage en x pour la pose des cubes 
        self.delta_y = 0.083 #(en mm) decalage en y pour la pose des cubes 
        self.correctif_pose=[0,0,0,0.0,-np.pi,0.0]#0.066/0.121

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
    
    def cam2base(self, objetCam, posePrise = None):
        objetCam = np.transpose(objetCam + [1])
        # si on veut la pose actuelle
        if posePrise == None:
            posePrise = self.robot_r.getActualTCPPose()
        T_cam2gripper = [[ 0.04853044,  0.99880257,  0.00618264,  0.10201555],
                        [-0.99542155,  0.047854,    0.08274014,  0.0217057 ],
                        [ 0.0823452,  -0.01016975,  0.99655198, -0.150],
                        [ 0.  ,        0.    ,      0.   ,       1.        ]]      
        T_gripper2base = create_matrice(posePrise)
        res = T_gripper2base @ T_cam2gripper @ objetCam
        return res[:3]

    def correction_pose(self,pose):
        for i in range(3):
            pose[3+i]+=self.correctif_pose[3+i]
        return pose
        


    def deconnexion(self): 
        self.robot_c.disconnect()

    def bouger(self, pos, speed=0.5, acceleration=0.3):
        self.connexion()
        self.robot_c.moveL(pos, speed, acceleration)
        self.deconnexion()

    def rangement(self, pince: Pince):
        #calcul pos_rangement en fonction de self.num_cube
        pos_rangement= self.calcul_pos_relative(self.delta_x * (self.num_cube//3), self.delta_y* (self.num_cube%3), pos=self.pos_depot_cube)
        # print(f"{pos_rangement=}")

        #bouger à pos_rangement (avec pos_intermediaire au dessus)
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement),1,5) #verif si z + ou -
        self.bouger(pos_rangement, 0.5, 0.3)  
        
        #lacher
        pince.lacher()
        
        #remonter
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement),1,5) #verif si z + ou -

        #maj compteur cube
        self.num_cube +=1
        # print(self.num_cube)


if __name__ == "__main__":
    robot = Robot()
    pince = Pince()
    robot.bouger(robot.pos_init, 3, 1)

    robot.bouger(robot.pos_cam_1, 3, 1)
    robot.bouger(robot.pos_cam_2, 3, 1)
    robot.bouger(robot.pos_cam_3, 3, 1)

    robot.bouger(robot.pos_init, 2, 0.3)

    robot.bouger(robot.pos_cam_4, 3, 1)
    robot.bouger(robot.pos_cam_5, 2, 0.3)

    robot.bouger(robot.pos_init, 2, 0.3)

    robot.bouger(robot.pos_cam_6, 2, 0.3)
    robot.bouger(robot.pos_init,2)
    # for _ in range(9) :
    #     robot.rangement(pince)
    # robot.rangement(pince) 

    
    # robot.bouger(robot.pos_init,2)
    # robot.robot_c.stopScript()

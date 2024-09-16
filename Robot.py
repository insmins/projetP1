"""
Classe avec fonctions utiles pour le robot
"""
# import
import rtde_receive
import rtde_control
# from Transfo import create_matrice
import numpy as np

class Robot :
    def __init__(self):
        # variable du robot
        self.num_cube = 0
        self.pos_init =[] # a def
        self.pos_depot_cube = [] # a def
        self.pos_nuage_point = [] #a def (voir si plusieurs)
        self.delta_x = 0.065 #(en mm) decalage en x pour la pose des cubes 
        self.delta_y = 0.065 #(en mm) decalage en y pour la pose des cubes 

        #move to pose initiale
        self.robot_c.moveL(self.pos_init, 0.5, 0.3)


    def connexion(self):
        self.robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
        self.robot_c = rtde_control.RTDEControlInterface("10.2.30.60")

    def bouger_relatif(self, dx=0, dy=0, dz=0, pos = None):
        if pos is None :
            pos = self.robot_r.getActualTCPPose()
        pos = [pos[0]+dx,pos[1]+dy,pos[2]+dz,pos[3],pos[4],pos[5]]

        # # move robot
        # self.robot_c.moveL(pos, 0.5, 0.3)

        # # disconnect
        # self.robot_c.disconnect()
        return pos
    
    # def cam2base(self, objetCam):
    #     objetCam = np.transpose(objetCam + [1])

    #     T_cam2gripper = np.load("../FinalTransforms/T_cam2gripper_Method_0.npz")['arr_0']
    #     print(T_cam2gripper )
    #     # T_cam2gripper[0, 3] = 0.100
    #     T_cam2gripper[1, 3] = 0.023
    #     T_cam2gripper[2, 3] = -0.210
    #     posePrise = self.robot_r.getActualTCPPose()
    #     T_gripper2base = create_matrice(posePrise)

    #     res = T_gripper2base @ T_cam2gripper @ objetCam
    #     print(f'{res=}')
    #     return res[:3]

    def deconnecter(self): # a tej ? 
        self.robot_c.stopScript()

    
    def rangement(self):
        #calcul pos_rangement en fonction de self.num_cube
        pos_rangement= self._bouger_relatif(self.delta_x * (self.num_cube//3), self.delta_y* (self.num_cube%3), pos=self.pos_depot_cube)

        #bouger à pos_rangement (avec pos_intermediaire au dessus)
        self.robot_c.moveL(self.bouger_relatif(dz=0.1, pos=self.pos_rangement)) #verif si z + ou -
        self.robot_c.moveL(pos_rangement, 0.5, 0.3)
        self.robot_c.moveL(self.bouger_relatif(dz=0.1, pos=self.pos_rangement)) #verif si z + ou -

        #maj compteur cube
        self.num_cube +=1

        # disconnect
        self.robot_c.disconnect()



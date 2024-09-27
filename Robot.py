"""
Classe avec fonctions utiles pour le robot
"""
# import
import rtde_receive
import rtde_control
from Transfo import create_matrice, matrice_to_pose
import numpy as np
from Pince import Pince
import time

class Robot :
    def __init__(self):
        # variable du robot
        self.num_cube = 0
        self.pos_init =[-0.27961,-0.11156, 0.23741, 0.135,-3.128, 0.144] 
        self.pos_depot_cube = [-0.48118,-0.26843, 0.06306, 0.082,-3.120, 0.114] #premiere position pour déposer un cube
        self.pos_cam_1 = [-0.22973, 0.06416, 0.29767, 0.059,-3.210, 0.160] #position de prise de photo
        self.pos_cam_2 = [-0.35594, 0.16063, 0.27617, 0.097, 2.538,-0.192] #position de prise de photo
        self.pos_cam_3 = [-0.14889,-0.02927, 0.20331, 0.196, 3.455, 0.276] #position de prise de photo
        self.pos_cam_4 = [-0.17349, 0.15271, 0.24970, 1.610, 2.644,-0.680] #position de prise de photo
        self.pos_cam_5 = [-0.21852, 0.08733, 0.25996, 2.090, 2.747,-0.771] #position de prise de photo
        self.pos_cam_6 = [-0.32933,-0.00430, 0.26788, 2.582,-2.630,-1.215] #position de prise de photo
        self.delta_x = 0.083 #(en mm) decalage en x pour la pose des cubes 
        self.delta_y = 0.083 #(en mm) decalage en y pour la pose des cubes 
        # self.correctif_pose=[0,0,0,0.0,-np.pi,0.0]#0.066/0.121


    def connexion(self):
        """Fonction pour se connecter au robot grâce à son IP"""
        self.robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
        self.robot_c = rtde_control.RTDEControlInterface("10.2.30.60")

    def calcul_pos_relative(self, dx=0, dy=0, dz=0, pos = None):
        """Calcul une pose à partir d'une autre et d'un changement donné"""
        if pos is None :
            pos = self.robot_r.getActualTCPPose()
        pos = [pos[0]+dx,pos[1]+dy,pos[2]+dz,pos[3],pos[4],pos[5]]
        return pos
    
    def cam2base(self, objetCam, pose = None):
        """Remet objetCam dans le référentiel du robot en fonction de la pose d'entrée"""
        objetCam = np.transpose(objetCam + [1])
        # si on veut la pose actuelle
        if pose == None:
            pose = self.robot_r.getActualTCPPose()
        T_cam2gripper = [[ 0.04853044,  0.99880257,  0.00618264,  0.10201555],
                        [-0.99542155,  0.047854,    0.08274014,  0.0217057 ],
                        [ 0.0823452,  -0.01016975,  0.99655198, -0.153],
                        [ 0.  ,        0.    ,      0.   ,       1.        ]]      
        T_gripper2base = create_matrice(pose)
        res = T_gripper2base @ T_cam2gripper @ objetCam
        return res[:3]


    def deconnexion(self): 
        """Déconnexion du robot"""
        self.robot_c.disconnect()

    def bouger(self, pos, speed=0.5, acceleration=0.3):
        """Déplacement du robot selon une pose donnéee"""
        self.connexion()
        self.robot_c.moveL(pos, speed, acceleration)
        self.deconnexion()

    def rangement(self, pince: Pince):
        """Dépot d'un cube à l'emplacement voulu"""
        #calcul pos_rangement en fonction de self.num_cube
        pos_rangement= self.calcul_pos_relative(self.delta_x * (self.num_cube//3), self.delta_y* (self.num_cube%3), pos=self.pos_depot_cube)

        #bouger à pos_rangement (avec pos_intermediaire au dessus)
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement),1,5) #verif si z + ou -
        self.bouger(pos_rangement, 0.5, 0.3)  
        
        #lacher
        pince.lacher()
        
        #remonter
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement),1,5) #verif si z + ou -

        #maj compteur cube
        self.num_cube +=1
    
    def rotation(self,gamma, beta,alpha): 
        """
        Calcul de la matrice de rotation 3x3 en fonction de alpha, beta et gamma.
        Alpha : rotation selon X (en degré)
        Beta : rotation selon y (en degré)
        Gamma : rotation selon z (en degré)
        """
        #conversion alpha, beta, gamma radian
        alpha=alpha*(np.pi/180)
        beta=np.pi+beta*(np.pi/180)
        gamma=gamma*(np.pi/180)
        #calcul des matrice de roation selon chaque
        Rx=np.asanyarray([[1,            0,             0],
                        [0,np.cos(gamma),-np.sin(gamma)],
                        [0,np.sin(gamma), np.cos(gamma)]])
        Ry=np.asanyarray([[np.cos(beta) ,0,np.sin(beta)],
                        [0            ,1,           0],
                        [-np.sin(beta),0,np.cos(beta)]])
        Rz=np.asanyarray([[np.cos(alpha),-np.sin(alpha),0],
                        [np.sin(alpha), np.cos(alpha),0],
                        [0            , 0            ,1]])
        return Rz @ Ry @ Rx

    def matrice_passage_normale(mat_rot,trans):
        """
        Créer la matrice de passage 4x4 grâce à la matrice de rotation et le vecteur translation
        """
        res=mat_rot.tolist()
        res.append([0,0,0,1])
        for i in range(len(trans)):
            res[i].append(trans[i])
        return np.asanyarray(res)



if __name__ == "__main__":
    robot = Robot()
    pince = Pince()

    # test des positions de  cam
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

    # # test des positions de rangement
    # for _ in range(9) :
    #     robot.rangement(pince)
    # robot.rangement(pince) 


    # # test bouger selon rotation
    # point=robot.pos_init[:3]    
    # alpha=0 #selon x
    # beta=0 # selon y
    # gamma=0 # selon z
    # # robot.bouger(pos,0.5)
    # mat4x4=robot.matrice_passage_normale(robot.rotation(gamma, beta, alpha),point)
    # # print("mat4x4 :\n",mat4x4)
    # pos=matrice_to_pose(mat4x4)
    # # print("pose",pos)
    # robot.bouger(pos,0.5)
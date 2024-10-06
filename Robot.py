"""
Nom du fichier : Robot.py
Auteur : Mattéo CAUX et Inès EL HADRI
Date : 2024-10-02
Description : Ce script contient la classe robot et l'algorithme effectuant toutes les actions
"""

# import
import rtde_receive
import rtde_control
from Transfo import create_matrice, matrice_to_pose
import numpy as np
from Pince import Pince

class Robot :
    """
    Classe avec fonctions utiles pour le robot
    """
    def __init__(self, IP = "10.2.30.60"):
        """initialisation

        Args:
            IP (str, optional): IP du robot. Defaults to "10.2.30.60".
        """
        # variable du robot
        self.num_cube = 0 #nombre de cube déposé (cf la fonction rangement)
        self.pos_init =[-0.27961,-0.11156, 0.23741, 0.135,-3.128, 0.144] #position de repos pour le robot
        self.pos_cam_1 = [-0.22973, 0.06416, 0.29767, 0.059,-3.210, 0.160] #position de prise de photo
        self.pos_cam_2 = [-0.35594, 0.16063, 0.27617, 0.097, 2.538,-0.192] #position de prise de photo
        self.pos_cam_3 = [-0.14889,-0.02927, 0.20331, 0.196, 3.455, 0.276] #position de prise de photo
        self.pos_cam_4 = [-0.17349, 0.15271, 0.24970, 1.610, 2.644,-0.680] #position de prise de photo
        self.pos_cam_5 = [-0.21852, 0.08733, 0.25996, 2.090, 2.747,-0.771] #position de prise de photo
        self.pos_cam_6 = [-0.32933,-0.00430, 0.26788, 2.582,-2.630,-1.215] #position de prise de photo
        self.pos_depot_cube = [-0.48118,-0.26843, 0.06306, 0.082,-3.120, 0.114] #premiere position pour déposer un cube
        self.delta_x = 0.083 #(en mm) decalage en x pour la pose des cubes 
        self.delta_y = 0.083 #(en mm) decalage en y pour la pose des cubes 
        self.ip = IP # IP du robot
        # matrice de changement entre la base de la cam et celle de la pince 
        self.T_cam2gripper = [[ 0.04853044,  0.99880257,  0.00618264,  0.10201555],
                        [-0.99542155,  0.047854,    0.08274014,  0.0217057 ],
                        [ 0.0823452,  -0.01016975,  0.99655198, -0.153],
                        [ 0.  ,        0.    ,      0.   ,       1.        ]] 

    def connexion(self):
        """Fonction pour se connecter au robot grâce à son IP"""
        self.robot_r = rtde_receive.RTDEReceiveInterface(self.ip)
        self.robot_c = rtde_control.RTDEControlInterface(self.ip)

    def deconnexion(self): 
        """Déconnexion du robot"""
        self.robot_c.disconnect()

    def bouger(self, pos, speed=0.5, acceleration=0.3):
        """
        Déplacement du robot selon une pose donnée avec connexion préalable et déconnexion à la fin de l'action.

        Args:
            pos (list[float]): position à laquelle le robot doit aller.
            speed (float, optional): vitesse du déplacement.
            acceleration (float, optional): acceleration pour le déplacement.
        """
        self.connexion()
        self.robot_c.moveL(pos, speed, acceleration)
        self.deconnexion()

    def calcul_pos_relative(self, dx=0, dy=0, dz=0, pos = None):
        """
        Calcul une pose à partir d'une autre et d'un changement donné.

        Args:
            dx (float, optional): variation selon l'axe x en mètre. Default to 0
            dy (float, optional): variation selon l'axe y en mètre. Default to 0
            dz (float, optional): variation selon l'axe z en mètre. Default to 0
            pos (list[float], optional): position à partir de laquelle on veut bouger. Default to pose actuelle

        Returns:
            list[float] : La nouvelle position calculée
        """
        # pose actuelle si pos non donné
        if pos is None :
            pos = self.robot_r.getActualTCPPose()
        pos = [pos[0]+dx,pos[1]+dy,pos[2]+dz,pos[3],pos[4],pos[5]] # calcul de la nouvelle pose
        return pos

    def rangement(self, pince: Pince):
        """
        Dépot d'un cube à l'emplacement de rangement voulu en fonction du numéro de cube.
        
        Args:
            pince (Pince): une instance de Pince.
        """
        #calcul pos_rangement en fonction de self.num_cube
        pos_rangement= self.calcul_pos_relative(self.delta_x * (self.num_cube//3), self.delta_y* (self.num_cube%3), pos=self.pos_depot_cube)

        #bouger à pos_rangement (avec pos_intermediaire au dessus)
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement),1,5) #verif si z + ou -
        self.bouger(pos_rangement, 0.5, 0.3)  
        
        #lacher
        pince.lacher()
        
        #remonter
        self.bouger(self.calcul_pos_relative(dz=0.1, pos=pos_rangement),1,5) #verif si z + ou -

        #update compteur cube
        self.num_cube +=1
    
    def cam2base(self, objetCam, pose = None):
        """
        Remet objetCam dans le référentiel du robot en fonction de la pose d'entrée.
        
        Args:
            objetCam (list[float]): les coordonnés du point à remettre dans la base du robot.
            pose (list[float], optional): la position du robot à partir de laquelle faire le changement de base. Default to pose actuelle
        
        Returns:
            numpy.ndarray: les coordonnées du point dans la base du robot.
        """
        objetCam = np.transpose(objetCam + [1])
        # pose actuelle si pos non donné
        if pose == None:
            pose = self.robot_r.getActualTCPPose()
        # calcul de la matrice de changement entre la base de la pince et celle du robot
        T_gripper2base = create_matrice(pose)
        res = T_gripper2base @ self.T_cam2gripper @ objetCam
        return res[:3]
    
    def rotation(self,gamma, beta,alpha): 
        """
        Calcul de la matrice de rotation 3x3 en fonction de alpha, beta et gamma.

        Args:
            Alpha (float): rotation selon X (en degré).
            Beta (float): rotation selon y (en degré).
            Gamma (float): rotation selon z (en degré).
        
        Returns:
            numpy.ndarray: matrice de rotation 3x3.
        """
        #conversion alpha, beta, gamma radian
        alpha=alpha*(np.pi/180)
        beta=beta*(np.pi/180)
        gamma=gamma*(np.pi/180)
        #calcul des matrice de rotation selon chaque axe
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

    def matrice_passage_normale(self,mat_rot,trans):
        """
        Créer la matrice de passage 4x4 grâce à la matrice de rotation et le vecteur translation.

        Args:
            mat_rot (numpy.array): La matrice de rotation 3x3.
            trans (list[float]): Le vecteur de translation.
        
        Returns:
            numpy.array: La matrice de passage 4x4.
        """
        res=mat_rot.tolist()
        res.append([0,0,0,1])
        for i in range(len(trans)):
            res[i].append(trans[i])
        return np.asanyarray(res)


if __name__ == "__main__":
    robot = Robot()
    pince = Pince()
    robot.bouger(robot.pos_init, 3, 1)

    """test des positions de cam"""
    # robot.bouger(robot.pos_cam_1, 3, 1)
    # robot.bouger(robot.pos_cam_2, 3, 1)
    # robot.bouger(robot.pos_cam_3, 3, 1)

    # robot.bouger(robot.pos_init, 2, 0.3)

    # robot.bouger(robot.pos_cam_4, 3, 1)
    # robot.bouger(robot.pos_cam_5, 2, 0.3)

    # robot.bouger(robot.pos_init, 2, 0.3)

    # robot.bouger(robot.pos_cam_6, 2, 0.3)
    # robot.bouger(robot.pos_init,2)

    """test des positions de rangement"""
    # for _ in range(9) :
    #     robot.rangement(pince)
    # robot.rangement(pince) 


    """test bouger selon rotation"""
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



    """test des matrices de changement de base"""
    # base d'un cube et son centre à  tester
    base=np.array([[-0.92547062, -0.31436589,  0.20930943],       [-0.36437526,  0.89362171, -0.26388958],       [-0.10360888, -0.32033473, -0.94156883]])
    centre=np.array([-0.2767818 ,  0.15665482,  0.0477021 ])   

    # on regarde si l'axe z est vers le haut 
    roty = 0
    if base[2][2] > 0:
        roty = 180

    # on transpose la base pour avoir les vecteurs en colonne, ce qui donne ma matrice de rotation 3x3
    base=np.transpose(base)
    print(f'{base=}')
    print(f'{centre=}')

    # si l'axe z est vers le haut, rotation de 180° selon y pour le mettre vers le bas 
    rot = robot.rotation(0, roty, 0)
    base = base @ rot 
    
    # print(f'{rot=}' )
    # print(f'{base=}')

    # création de la matrice de passage 4x4 avec la matrice de rotation et le vecteur translation (ici les coordonnées de centre)
    mat_passage=robot.matrice_passage_normale(base, centre)
    # print(f'{mat_passage=}')

    # creation du points de prise (M) et d'un point au dessus de prise (N)
    M = [0]*3
    M[2] = -0.013
    N= [0]*3
    N[2]=-0.2

    # creation des matrices de passages et des poses pour aller à ces points.
    M = mat_passage @ np.transpose(M+[1])
    N = mat_passage @ np.transpose(N+[1])
    mat_M = robot.matrice_passage_normale(base,np.transpose(M[:3]))
    mat_N = robot.matrice_passage_normale(base,np.transpose(N[:3]))
    pose_cube=matrice_to_pose(mat_M)
    pose_dessus_cube = matrice_to_pose(mat_N)

    # prise du cube
    robot.bouger(pose_dessus_cube, 0.3)
    robot.bouger(pose_cube)
    pince.prise()
    robot.bouger(pose_dessus_cube)

    # passage par robot.pos_init pour éviter les singularités 
    robot.bouger(robot.pos_init)

    # dépôt du cube
    robot.rangement(pince)

    
    # retour à la position initiale 
    robot.bouger(robot.pos_init)
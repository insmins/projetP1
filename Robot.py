"""
Classe avec fonctions utiles pour le robot
"""
# import
import rtde_receive
import rtde_control
from Transfo import create_matrice, matrice_to_pose, matrice_rotation_3x3
import numpy as np
from Pince import Pince
import time
from Camera import Camera

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
        beta=beta*(np.pi/180)
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

    def matrice_passage_normale(self,mat_rot,trans):
        """
        Créer la matrice de passage 4x4 grâce à la matrice de rotation et le vecteur translation
        """
        res=mat_rot.tolist()
        res.append([0,0,0,1])
        for i in range(len(trans)):
            res[i].append(trans[i])
        return np.asanyarray(res)



if __name__ == "__main__":
    from cube import Cube
    robot = Robot()
    pince = Pince()
    robot.bouger(robot.pos_init, 3, 1)

    # # test des positions de  cam
    # robot.bouger(robot.pos_cam_1, 3, 1)
    # robot.bouger(robot.pos_cam_2, 3, 1)
    # robot.bouger(robot.pos_cam_3, 3, 1)

    # robot.bouger(robot.pos_init, 2, 0.3)

    # robot.bouger(robot.pos_cam_4, 3, 1)
    # robot.bouger(robot.pos_cam_5, 2, 0.3)

    # robot.bouger(robot.pos_init, 2, 0.3)

    # robot.bouger(robot.pos_cam_6, 2, 0.3)
    # robot.bouger(robot.pos_init,2)

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

    # base = [[ 0.11318712, -0.05391406,  0.99210985], [-0.46113575,  0.88161765,  0.10051933], [ 0.98480458,  0.13845839, -0.10482946]]
    # base = [[-0.04793601, -0.30520967, -0.95107791], [-0.69461502,  0.69442563, -0.18783776], [ 0.71778257,  0.65162901, -0.24529127]]
    # centre = [-0.28892185, 0.13279266, 0.05123627]
    # base = [[ 0.12309199, -0.05049066,  0.99111001], [-0.49897492, -0.86643296,  0.01783156], [ 0.85745872, -0.4973782 , -0.13183123]]
    # centre = [-0.29210161, 0.13408612, 0.04832842]
    # base = [[ 0.05669472, -0.11661385,  0.99155782], [ 0.85654028, -0.50458508, -0.10831735], [0.51383997, 0.85492922, 0.07116536]] 
    # centre = [-0.2891642,  0.1290486,  0.04429216]
    # base = matrice_rotation_3x3(np.asarray(robot.pos_init[3:]))
    # centre = robot.pos_init[:3]
    # base = [[-0.06402565,  0.04122396, -0.99709644], [ 0.8782171 , -0.47219876, -0.07591475], [-0.4738714 , -0.88057377, -0.00597818]] 
    # base = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
    # centre = [-0.27295247, 0.17494993, 0.05353815]
    # base=cube.creer_base_directe(base[0], base[1], base[2])

    cube=Cube()
    cam=Camera()

    base, centre = cube.main(cam, robot)
    # print(base, centre)
    # base = [[ 0.96153612,  0.25975672, -0.08930141], [np.float64(-0.26091364699558284), np.float64(0.9653602545721748), np.float64(-0.0019098962825697936)], [0.08571192, 0.02513639, 0.99600283]] 
    # centre = [-0.28378925, 0.16210027, 0.0496929]

    # base=np.array([[-0.92547062, -0.31436589,  0.20930943],       [-0.36437526,  0.89362171, -0.26388958],       [-0.10360888, -0.32033473, -0.94156883]])
    # centre=np.array([-0.2767818 ,  0.15665482,  0.0477021 ])   

    roty = 0
    if base[2][2] > 0:
        roty = 180

    base=np.transpose(base)
    print(f'{base=}')
    print(f'{centre=}')

    rot = robot.rotation(0, roty, 0)
    base = base @ rot 
    
    # print(f'{rot=}' )
    # print(f'{base=}')


    mat_passage=robot.matrice_passage_normale(base, centre)
    # print(f'{mat_passage=}')

    M = [0]*3
    M[2] = -0.013
    N= [0]*3
    N[2]=-0.2

    M = mat_passage @ np.transpose(M+[1])
    N = mat_passage @ np.transpose(N+[1])
    # print(f'{M=}')
    
    mat_M = robot.matrice_passage_normale(base,np.transpose(M[:3]))
    mat_N = robot.matrice_passage_normale(base,np.transpose(N[:3]))

    # print(f'{mat_M=}')
    pose_cube=matrice_to_pose(mat_M)
    # pose_cube[2] += 0.01325
    pose_dessus_cube = matrice_to_pose(mat_N)
    # pose_dessus_cube[2] += 0.2
    # pose_dessus_cube[4]+=np.pi  
    # pose_dessus_cube[3]=0.135
    # print(f'{pose_dessus_cube=}')
    robot.bouger(pose_dessus_cube, 0.3)
    robot.bouger(pose_cube)
    pince.prise()
    robot.bouger(pose_dessus_cube)
    robot.bouger(robot.pos_init)
    robot.rangement(pince)
    robot.bouger(robot.pos_init)
    
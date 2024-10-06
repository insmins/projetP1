"""
Nom du fichier : ProjetP1.py
Auteur : Mattéo CAUX et Inès EL HADRI
Date : 2024-10-04
Description : Ce script réalise les différentes étapes du projet, de la prise de photo au dépôt du cube.
"""

# import
from Robot import Robot
from Pince import Pince
from Camera import Camera
from cube import Cube
import time 
from Transfo import matrice_to_pose
import numpy as np

temps_debut=time.time()

# création des instances de classes
robot = Robot()
pince = Pince()
cube=Cube(0.05)
cam=Camera()

# déplacement du robot à sa position initiale
robot.bouger(robot.pos_init, 3, 1)


####### entrée de la boucle while pour tester la présence d'un cube (non codé) ##########
# while fonction_qui_teste_cube :  

# obtention du centre et de la base du cube
base, centre = cube.main(cam, robot)

# on regarde si l'axe z est vers le haut 
roty = 0
if base[2][2] > 0:
    roty = 180

# on transpose la base pour avoir les vecteurs en colonne, ce qui donne ma matrice de rotation 3x3
base=np.transpose(base)
# print(f'{base=}')

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
M[2] = -0.005
N= [0]*3
N[2]=-0.2

# creation des matrices de passages et des poses pour aller à ces points.
M = mat_passage @ np.transpose(M+[1])
N = mat_passage @ np.transpose(N+[1])    
mat_M = robot.matrice_passage_normale(base,np.transpose(M[:3]))
mat_N = robot.matrice_passage_normale(base,np.transpose(N[:3]))
pose_cube = matrice_to_pose(mat_M)
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

############## sortie du while cube #########################

# calcul du temps d'éxécution
temps_fin=time.time()
delta_temps=temps_fin-temps_debut
print(delta_temps)
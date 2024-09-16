# import socket
# import time
# from Robot import Robot

# # rtde_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
# # rtde_c = rtde_control.RTDEControlInterface("10.2.30.60")

# IP_robot = "10.2.30.60"
# port_dashboard = 29999  # Pour la connexion via socket à l'IHM
# port_robot = 30002  # Pour la connexion via socket au robot lui-même

# # socket permet d'envoyer des commandes script à faire exécuter à l'IHM ou au robot
# robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# robot.connect((IP_robot, port_robot))
# dashboard.connect((IP_robot, port_dashboard))
# def action_pince(action):



#     if action == "prise":  # Changement de valeur de la sortie suivant le mouvement de pince à effectuer
#         robot.send(("set_standard_digital_out(0,True)" + "\n").encode('utf8'))
#     elif action == "lacher":
#         robot.send(("set_standard_digital_out(0,False)" + "\n").encode('utf8'))

#     dashboard.send(("stop" + "\n").encode('utf8'))  # On arrête le programme local si celui-ci est en train de tourner
#     time.sleep(1)
#     dashboard.send(("play" + "\n").encode('utf8'))  # Lancement d'un programme local sur l'IHM qui commande l'ouverture et la fermeture de la pince
#     time.sleep(4)
#     dashboard.send(("stop" + "\n").encode('utf8'))  # On arrête à nouveau le programme local

#     robot.close()  
#     # rtde_c.reconnect() # On reconnecte le robot en RTDEControl après avoir envoyé des requêtes via socket

# action_pince("lacher")

# # au tour du robot
# robot = Robot()

# pos =robot.bouger_relatif(dy=-0.1)
# robot.robot_c.moveL(pos, 0.5, 0.3)


# # refermer la pince apres
# robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# robot.connect((IP_robot, port_robot))
# dashboard.connect((IP_robot, port_dashboard))
# action_pince("prise")

from Pince import Pince
from Robot import Robot

pince= Pince()
robot = Robot()

pince.connexion()
pince.lacher()

# au tour du robot

robot.connexion()
robot.bouger_relatif(dy=0.1)


#re pince 
pince.connexion()
pince.prise()

#re robot
robot.connexion()
robot.bouger_relatif(dy=-0.1)

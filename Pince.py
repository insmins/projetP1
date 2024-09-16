import socket
import time

class Pince :
    def __init__(self):
        self.IP_robot = "10.2.30.60"
        self.port_dashboard = 29999  # Pour la connexion via socket à l'IHM
        self.port_robot = 30002  # Pour la connexion via socket au robot lui-même

    def connexion(self):
        # socket permet d'envoyer des commandes script à faire exécuter à l'IHM ou au robot
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot.connect((self.IP_robot, self.port_robot))
        self.dashboard.connect((self.IP_robot, self.port_dashboard))


    def _action_pince(self, action):
        if action == "prise":  # Changement de valeur de la sortie suivant le mouvement de pince à effectuer
            self.robot.send(("set_standard_digital_out(0,True)" + "\n").encode('utf8'))
        elif action == "lacher":
            self.robot.send(("set_standard_digital_out(0,False)" + "\n").encode('utf8'))

        self.dashboard.send(("stop" + "\n").encode('utf8'))  # On arrête le programme local si celui-ci est en train de tourner
        time.sleep(1)
        self.dashboard.send(("play" + "\n").encode('utf8'))  # Lancement d'un programme local sur l'IHM qui commande l'ouverture et la fermeture de la pince
        time.sleep(4)
        self.dashboard.send(("stop" + "\n").encode('utf8'))  # On arrête à nouveau le programme local

        self.robot.close()
    
    def prise(self):
        self._action_pince("prise")

    def lacher(self):
        self._action_pince("lacher")
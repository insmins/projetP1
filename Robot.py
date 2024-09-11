"""
Classe avec fonctions utiles pour le robot
"""
# import
import rtde_receive
import rtde_control

class Robot :
    def __init__(self):
        # init communication robot
        self.robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
        self.robot_c = rtde_control.RTDEControlInterface("10.2.30.60")

    def bouger_relatif(self, dx=0, dy=0, dz=0):
        pos = self.robot_r.getActualTCPPose()
        pos = [pos[0]+dx,pos[1]+dy,pos[2]+dz,pos[3],pos[4],pos[5]]
        return pos


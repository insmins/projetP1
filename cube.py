from Robot import Robot
from Camera import Camera
import numpy as np
import polyscope as ps
import open3d as o3d

class Cube:
    def __init__(self):
        self.chemin_dossier="./photo_cam/"
        self.maxi_points=None
        self.len = 2
        self.x = np.asanyarray(np.linspace(0, 0.065, len))
        self.y = self.x.copy()
        self.z = self.x.copy()
        self.cube = []


        for i in range(len):
            for j in range(len):
                self.cube.append([self.x[0], self.y[i], self.z[j]])
                self.cube.append([self.x[-1], self.y[i], self.z[j]])
                self.cube.append([self.x[i], self.y[0], self.z[j]])
                self.cube.append([self.x[i], self.y[-1], self.z[j]])
                self.cube.append([self.x[i], self.y[j], self.z[0]])
                self.cube.append([self.x[i], self.y[j], self.z[-1]])

        self.cube = np.asanyarray(cube)

        self.centre = np.array([[np.mean(self.x), np.mean(self.y), np.mean(self.z)]])


    def create_points(self, cam,robot):
        # init
        robot.bouger(robot.pos_init,2)

        # positions de prise de vue
        pos_prise = [robot.pos_cam_1, robot.pos_cam_2, robot.pos_cam_3, robot.pos_cam_4, robot.pos_cam_5, robot.pos_cam_6]

        for i in range(6):
            robot.bouger(robot.pos_init, 2)
            robot.bouger(pos_prise[i])
            cam.updateCam()
            # depth_image = np.asanyarray(cam.aligned_depth_frame.get_data())

            xyz = cam.create_xyz()
            print(len(xyz))
            pos = cam.positions_xyz(xyz)
            np.savetxt(self.chemin_dossier+f"uncube_cam_{i}.txt", pos)
        robot.bouger(robot.pos_init, 2)

    def creer_maxi_point(self,robot):
        # CREER LA MAXI LISTE de points (prend du temps)
        Points = [np.loadtxt(self.chemin_dossier+"uncube_cam_0.txt"), np.loadtxt(self.chemin_dossier+"uncube_cam_1.txt"), np.loadtxt(self.chemin_dossier+"uncube_cam_2.txt"), np.loadtxt(self.chemin_dossier+"uncube_cam_3.txt")]
        poscam = [robot.pos_cam_1, robot.pos_cam_2, robot.pos_cam_3, robot.pos_cam_4]

        maxi_points = []
        for i, points in enumerate(Points):
            for point in points :
                maxi_points.append(robot.cam2base(point.tolist(), poscam[i]))

        self.maxi_points = np.asanyarray(maxi_points)

        np.savetxt(self.chemin_dossier+"maxipoints.txt", maxi_points)
    
    def load_maxi_points(self):
        # load la maxi liste au lieu de la créer
        self.maxi_points = np.loadtxt(self.chemin_dossier+"maxipoints.txt") 

    def get_maxi_points(self):
        if self.maxi_points is None:
            print("maxi points est vide, erreur, au revoir")
            exit()
        else:
            return self.maxi_points
    
    def enlever_plateau(self):
        # enlever les points plus bas que le plateau
        maxi_points=self.get_maxi_points()
        asupp = []
        for i, point in enumerate(maxi_points):
            if point[2] < 0.014: # 0.014 est une valeur juste en dessous de la table
                asupp.append(i)

        self.maxi_points = np.delete(maxi_points, asupp, axis = 0)

    def tourner(self,alpha, beta):
        a = self.centre[0]
        b = self.centre[1]
        c = self.centre[2]
        return np.array([
            a*np.cos(alpha)*np.cos(beta) + b*np.sin(alpha)*np.cos(beta) + c*np.sin(beta),
            b*np.cos(alpha) - a*np.sin(alpha),
            - a*np.cos(alpha)*np.sin(beta) - b*np.sin(alpha)*np.sin(beta) + c*np.cos(beta)])
                

    def translater(self,point, ):
        [x, y, z] = point
        return np.asanyarray([self.centre[0] + x, self.centre[1] + y, self.centre[2] + z])


    def tourner_cube(self, alpha, beta):
        nouveau_cube = self.cube.copy()
        for p in nouveau_cube:
            a = p[0]
            b = p[1]
            c = p[2]
            p[0] = a*np.cos(alpha)*np.cos(beta) + b*np.sin(alpha)*np.cos(beta) + c*np.sin(beta)
            p[1] = b*np.cos(alpha) - a*np.sin(alpha)
            p[2] = - a*np.cos(alpha)*np.sin(beta) - b*np.sin(alpha)*np.sin(beta) + c*np.cos(beta)
        return nouveau_cube

    def translater_cube(self, point):
        [x, y, z] = point
        return np.asanyarray([[c[0] + x, c[1] + y, c[2] + z] for c in self.cube])

        

if __name__=="__main__":
    cube=Cube()
    robot=Robot()

    """
    visualize maxi points
    """
    #create maxi_points
    # cube.creer_maxi_point(robot)
    #load maxi_points
    cube.load_maxi_points()
    #enlever plateau
    cube.enlever_plateau()
    #get maxi_points
    maxi_points=cube.get_maxi_points()
    ps.init()
    ps_cloud = ps.register_point_cloud("my points", maxi_points)
    ps.show()

    """
    autre
    """

"""
Nom du fichier : Cube.py
Auteurs : Caux Mattéo, El Hadri Inès
Date : 2024-10-02
Description : classe permettant de reconnaitre un cube dans un nuage de points, et script de test
"""

# imports
from Robot import Robot
from Camera import Camera
import numpy as np
import polyscope as ps
import open3d as o3d

class Cube:
    """ classe permettant de reconnaitre un cube dans un nuage de points """
    def __init__(self, taille_cube=0.05):
        """initialisation

        Args:
            taille_cube (float, optional): taille d'une arête du cube. Defaults to 0.065.
        """
        self.chemin_dossier="./photo_cam/"
        self.maxi_points=None
        self.len_points_cube = 2
        self.taille_cube = taille_cube

        # création d'un cube : ses 8 coins et le centre du cube
        self.x = np.asanyarray(np.linspace(0, taille_cube, self.len_points_cube))
        self.y = self.x.copy()
        self.z = self.x.copy()
        self.cube = []
        self.cam_points = [None]*6


        for i in range(self.len_points_cube):
            for j in range(self.len_points_cube):
                self.cube.append([self.x[0], self.y[i], self.z[j]])
                self.cube.append([self.x[-1], self.y[i], self.z[j]])
                self.cube.append([self.x[i], self.y[0], self.z[j]])
                self.cube.append([self.x[i], self.y[-1], self.z[j]])
                self.cube.append([self.x[i], self.y[j], self.z[0]])
                self.cube.append([self.x[i], self.y[j], self.z[-1]])

        self.cube = np.asanyarray(self.cube)
        self.centre = np.array([np.mean(self.x), np.mean(self.y), np.mean(self.z)])


    def create_points(self, cam: Camera, robot: Robot, save=False):
        """creer chaque nuage de points selon positions de camera

        Args:
            cam (Camera): caméra de la classe caméra pour prendre les photos
            robot (Robot): robot de la classe Robot pour bouger dans les positions de prise de vue
            save (bool, optional): activer la sauvegarde des nuages de points (numpy array) en .txt . Defaults to False.
        """
        # positions de prise de vue
        pos_prise = [robot.pos_cam_1, robot.pos_cam_2, robot.pos_cam_3, robot.pos_cam_4, robot.pos_cam_5, robot.pos_cam_6]

        for i in range(6):
            # init puis position camera
            robot.bouger(robot.pos_init, 2,2)
            robot.bouger(pos_prise[i],2,2)
            
            # prendre photo
            cam.updateCam()
            xyz = cam.create_xyz()
            # print(len(xyz))
            pos = cam.positions_xyz(xyz)

            # sauvegarder les points
            self.cam_points[i] = pos
            if save:
                np.savetxt(self.chemin_dossier+f"uncube_cam_{i}.txt", pos)

        # init
        robot.bouger(robot.pos_init, 2,2)

    def create_maxi_points(self, robot, load=False, save=False):
        """CREER LA MAXI LISTE de points (prend du temps)
        Fusionne tous les nuages de points en les rapportant dans le repère du robot

        Args:
            robot (Robot): robot ayant pris les prises de vue pour convertir toutes les positions dans la base du robot
            load (bool, optional): charger un fichier txt d'un numpy array au lieu d'utiliser les points enregistrés. Defaults to False.
            save (bool, optional): sauvegarder la liste complète des points ramenés dans la base du robot. Defaults to False.
        """

        # Load points
        if load:
            self.cam_points = [np.loadtxt(self.chemin_dossier+"uncube_cam_0.txt"), np.loadtxt(self.chemin_dossier+"uncube_cam_1.txt"), np.loadtxt(self.chemin_dossier+"uncube_cam_2.txt"), np.loadtxt(self.chemin_dossier+"uncube_cam_3.txt")]
        
        poscam = [robot.pos_cam_1, robot.pos_cam_2, robot.pos_cam_3, robot.pos_cam_4, robot.pos_cam_5, robot.pos_cam_6]

        maxi_points = []
        for i, points in enumerate(self.cam_points):
            for point in points :
                maxi_points.append(robot.cam2base(point, poscam[i]))

        self.maxi_points = np.asanyarray(maxi_points)

        if save:
            np.savetxt(self.chemin_dossier+"maxipoints.txt", self.maxi_points)
    
    def load_maxi_points(self):
        """charge le fichier texte de "maxipoints" (numpy array) pour effectuer des tests
        """
        self.maxi_points = np.loadtxt(self.chemin_dossier+"maxipoints.txt") 
    
    def enlever_plateau(self):
        """enlever les points qui ne sont pas dans la zone de prise d'un cube (environnement)
        """
        # Filtrage des points dans la zone de travail
        xmin = -0.54
        xmax = -0.09
        ymin = -0.03
        ymax = 0.27
        zmin = 0.030 # Avec table enlevée
        # zmin = 0.014 # Avec table incluse
        self.maxi_points = np.array([p for p in self.maxi_points if p[0] > xmin and p[0] < xmax and p[1] > ymin and p[1] < ymax and p[2] > zmin])


    def tourner(self,alpha, beta):
        """effectue la rotation alpha, beta du centre du cube afin d'obtenir ses nouvelles coordonnées

        Args:
            alpha (float): angle de rotation selon x
            beta (float): angle de rotation selon y

        Returns:
            np.array: Nouvelle position de cube, tourné
        """
        a = self.centre[0]
        b = self.centre[1]
        c = self.centre[2]
        return np.array([
            a*np.cos(alpha)*np.cos(beta) + b*np.sin(alpha)*np.cos(beta) + c*np.sin(beta),
            b*np.cos(alpha) - a*np.sin(alpha),
            - a*np.cos(alpha)*np.sin(beta) - b*np.sin(alpha)*np.sin(beta) + c*np.cos(beta)])
                
    def translater(self,vecteur,point):
        """effectue la translation du point selon le vecteur afin d'obtenir ses nouvelles coordonnées

        Args:
            vecteur (list): vecteur à 3 dimensions
            point (list): coordonnées du point à déplacer

        Returns:
            np.array: coordonnées du point déplacé
        """
        [x, y, z] = vecteur
        return np.asanyarray([point[0] + x, point[1] + y, point[2] + z])

    def tourner_cube(self, alpha, beta):
        """tourner tous les points du cube

        Args:
            alpha (float): angle selon x
            beta (float): angle selon y

        Returns:
            list: nouveau cube tourné
        """
        nouveau_cube = self.cube.copy()
        for p in nouveau_cube:
            a = p[0]
            b = p[1]
            c = p[2]
            p[0] = a*np.cos(alpha)*np.cos(beta) + b*np.sin(alpha)*np.cos(beta) + c*np.sin(beta)
            p[1] = b*np.cos(alpha) - a*np.sin(alpha)
            p[2] = - a*np.cos(alpha)*np.sin(beta) - b*np.sin(alpha)*np.sin(beta) + c*np.cos(beta)
        return nouveau_cube

    def translater_cube(self, vecteur):
        """translater tous les points du cube

        Args:
            vecteur (list): vecteur de déplacement à 3 dimensions

        Returns:
            np.array: nouveau cube translaté
        """
        [x, y, z] = vecteur
        return np.asanyarray([[c[0] + x, c[1] + y, c[2] + z] for c in self.cube])
    
    def gramschmit(self, e1, e2, e3):
        """algorithme de Gram-Schmidt pour orthonormer une famille libre

        Args:
            e1 (list): vecteur 1
            e2 (list): vecteur 2
            e3 (list): vecteur 3

        Returns:
            tuple: les 3 nouveaux vecteurs
        """
        u1 = e1
        u2 = e2 - np.dot(e2, u1)*u1
        u3 = e3 - np.dot(e3, u1)*u1 - np.dot(e3, u2)*u2

        u1 = u1 / np.linalg.norm(u1)
        u2 = u2 / np.linalg.norm(u2)
        u3 = u3 / np.linalg.norm(u3)

        return u1, u2, u3
    
    def create_pointcloud3d(self, voxel_size=0.002):
        """crée un pointcloud open3d et effectue des traitements dessus

        Args:
            voxel_size (float, optional): taille du voxel pour supprimer des points. Defaults to 0.003.
        """
        self.pcl = o3d.geometry.PointCloud()
        self.pcl.points = o3d.utility.Vector3dVector(self.maxi_points)

        # supprimer les points aberrants
        nn = 16
        std_multiplier = 10
        filtered_pcl = self.pcl.remove_statistical_outlier(nn, std_multiplier)
        self.pcl = filtered_pcl[0]

        # voxel downsampling
        pcl_downsampled = self.pcl.voxel_down_sample(voxel_size=voxel_size)
        self.pcl = pcl_downsampled

        # normal calculation
        # self.pcl.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    def ransac_cube(self, points, num_iterations=1000):
        """Calcule le meilleur cube parmi num_iterations de cubes aléatoires en utilisant
        une version modifiée de la méthode Ransac

        Args:
            points (array): nuage de points
            num_iterations (int, optional): nombre d'itérations pour trouver le meilleur match. Defaults to 1000.

        Returns:
            tuple: meilleurs paramètres (point d'un coin du cube, angle alpha, angle beta),
                nuage de points contenant les points appartenant au cube trouvé
                coordonnées du centre du cube
        """
        # calcul des valeurs minimale et maximale de distance entre les points et le centre. Marge de 4%
        threshold_max = 1.04 * self.taille_cube * np.sqrt(3)/2  # Distance entre centre du cube et coin
        threshold_min = 0.96 * self.taille_cube / 2  # Distance entre centre du cube et centre d'une face


        # contiennent le meilleur match
        best_params = None
        best_number = -1
        best_centre = None
        best_inliers = None

        for _ in range(num_iterations):
            # un cube = un point, un alpha, un beta
            sample = points[np.random.randint(0, points.shape[0])]
            alpha = np.random.uniform(0, 2*np.pi)
            beta = np.random.uniform(0, 2*np.pi)

            # calcul de la position du centre du cube
            centre = self.translater(sample, self.tourner(alpha, beta))

            n = 0
            inliers = []
            for i, p in enumerate(points):
                # calcul de la distance entre chaque point et le centre du cube
                dist = np.sqrt((p[0]-centre[0])**2 + (p[1]-centre[1])**2 + (p[2]-centre[2])**2)

                # threshold max et min = on regarde si le point est entre le point le plus éloigné
                # et le moins éloigné du centre (entre le centre d'une face = threshold_min, et un coin du cube = threshold_max)
                if dist < threshold_max and dist > threshold_min:
                    inliers.append(i)
                    n += 1
            
            if n > best_number or best_number == -1:
                best_number = n
                best_params = [sample, alpha, beta]
                best_centre = centre
                best_inliers = inliers
        return best_params, best_inliers, best_centre
    
    def angle_matching(self, inliers):
        """le calcul du Ransac donne souvent une orientation peu précise, 
        cet algorithme permet de trouver une rotation optimale pour le cube

        Args:
            inliers (list): liste des points du cube
        """

        outliersplane_cloud = self.pcl.select_by_index(inliers)

        self.moyennes_normales = []
        while len(self.moyennes_normales) != 3:
            _, inliersplane = outliersplane_cloud.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=1000)
            inliersplane_cloud = outliersplane_cloud.select_by_index(inliersplane)
            outliersplane_cloud = outliersplane_cloud.select_by_index(inliersplane, invert=True)

            # compute the mean of all the normals in the plane
            inliersplane_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            normales = np.asarray(inliersplane_cloud.normals)

            normales = [np.sign(np.dot(normales[i], normales[0])) * normales[i]  for i in range(normales.shape[0])]
            inliersplane_cloud.normals = o3d.utility.Vector3dVector(np.array(normales))
            moy_norm = np.mean(inliersplane_cloud.normals, axis=0)
            moy_norm = moy_norm / np.linalg.norm(moy_norm)

            angles = [np.acos(np.dot(moy_norm, self.moyennes_normales[i])) for i in range(len(self.moyennes_normales))]

            append = True
            for a in angles:
                if a < np.pi/2 - np.pi/6 or a > np.pi/2 + np.pi/6:
                    append = False


            if append:
                self.moyennes_normales.append(moy_norm)

            # expliquer todo
            if len(outliersplane_cloud.points) < 3:
                if len(self.moyennes_normales) == 2:
                    self.moyennes_normales.append(np.cross(self.moyennes_normales[0], self.moyennes_normales[1]))
                else:
                    raise RuntimeError("Cube non trouvé")


    def better_vecteur(self):
        """transforme les vecteurs du cube en une base orthonormée DIRECTE

        Returns:
            list: base directe de 3 vecteurs à 3 dimensions
        """
        # faire gramschmidt pour orthonormer la base
        u1, u2, u3 = self.gramschmit(self.moyennes_normales[0], self.moyennes_normales[1], self.moyennes_normales[2])
        return self.creer_base_directe(u1, u2, u3)
    
    def creer_base_directe(self, u1, u2, u3):
        """transforme une famille quelconque en une base orthonormée DIRECTE

        Returns:
            list: base directe de 3 vecteurs à 3 dimensions
        """
        base_directe=[None]*3

        # remettre la base dans le bon ordre (plus proche de x, plus proche de y, plus proche de z)
        VECTEUR = 0
        vects = [u1.copy(), u2.copy(), u3.copy()]
        for i, u in enumerate(vects):
            if np.abs(np.dot(u, [0.0, 0, 1])) > np.abs(np.dot(vects[VECTEUR], [0.0, 0, 1])):
                VECTEUR = i
        base_directe[2]=vects[VECTEUR]
        vects.pop(VECTEUR)

        VECTEUR = 0
        for i, u in enumerate(vects):
            if np.abs(np.dot(u, [1.0, 0, 0])) > np.abs(np.dot(vects[VECTEUR], [1.0, 0, 0])):
                    VECTEUR = i
        base_directe[0]=vects[VECTEUR]
        vects.pop(VECTEUR)


        base_directe[1]=vects[0]

        # vérifier que c'est direct
        if np.linalg.det(base_directe)<0:
            base_directe[1]= [-x for x in base_directe[1]]

        return base_directe
    
    def main(self, cam, robot):
        """toutes les fonctions à faire dans le bon ordre

        Args:
            cam (Camera): caméra prenant les photos
            robot (Robot): robot effectuant le mouvement

        Returns:
            tuple: base du cube trouvée, et centre du cube
        """
        self.create_points(cam, robot)
        self.create_maxi_points(robot)
        self.enlever_plateau()
        self.create_pointcloud3d(voxel_size=0.001)
        _, inliers, CENTRE = self.ransac_cube(np.asanyarray(self.pcl.points), num_iterations=3000)

        inlier_cloud = self.pcl.select_by_index(inliers)

        self.angle_matching(inliers)

        centre_cloud = o3d.geometry.PointCloud()
        centre_cloud.points = o3d.utility.Vector3dVector(np.array([CENTRE, CENTRE, CENTRE]))
        centre_cloud.normals = o3d.utility.Vector3dVector(np.array(self.moyennes_normales)*10)
        o3d.visualization.draw_geometries([inlier_cloud, centre_cloud])

        BASE = self.better_vecteur()
        return BASE, CENTRE


           

        

if __name__=="__main__":
    cube=Cube()
    robot=Robot()
    cam=Camera()

    # obtention du centre et de la base du cube
    base, centre = cube.main(cam, robot)
    print(base, centre)

    # visualisation du cube et de sa base
    pcl_center=o3d.geometry.PointCloud()
    pcl_center.points=o3d.utility.Vector3dVector(np.array([centre,centre,centre]))
    pcl_center.normals=o3d.utility.Vector3dVector(np.array(base))

    o3d.visualization.draw_geometries([pcl_center,cube.pcl])




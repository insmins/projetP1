"""
Nom du fichier : Camera.py
Auteurs : Caux Mattéo, El Hadri Inès
Date : 2024-10-03
Description : classe permettant de communiquer avec la caméra Intel Realsense
"""

# imports
import time
import cv2
import pyrealsense2 as rs
import numpy as np


class Camera:
    """classe contenant les fonctions nécessaires pour communiquer avec la caméra intel realsense
    """
    def __init__(self):
        # Create a pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        # Get stream profile and camera intrinsics
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.color_intrinsics = color_profile.get_intrinsics()
        self.color_extrinsics = color_profile.get_extrinsics_to(color_profile)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.xyz =[]

    def updateCam(self):
        """Prise d'une photo et renvoie les différentes données recues.
        Les stocke aussi dans des variables de la classe

        Returns:
            tuple: frames, aligned_frames, aligned_depth_frame, color_frame
        """
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align.process(self.frames)

        # Get aligned frames
        self.aligned_depth_frame = self.aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        self.color_frame = self.aligned_frames.get_color_frame()
        return self.frames, self.aligned_frames, self.aligned_depth_frame, self.color_frame

    def mask_jaune(self, hsv_img):
        """Réalise un masque jaune sur l'image hsv

        Args:
            hsv_img (list): image en format hsv

        Returns:
            list: masque de l'image contenant le Jaune
        """
        # define range of yellow color in HSV
        lower_hsv = np.array([20, 110, 50])
        higher_hsv = np.array([50, 255, 255])

        # generating mask for yellow color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))

    def mask_vert(self, hsv_img):
        """Réalise un masque jaune sur l'image hsv

        Args:
            hsv_img (list): image en format hsv

        Returns:
            list: masque de l'image contenant le Vert
        """
        # define range of green color in HSV
        lower_hsv = np.array([50, 150, 50])
        higher_hsv = np.array([85, 255, 255])

        # generating mask for green color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))

    def mask_rouge(self, hsv_img):
        """Réalise un masque rouge sur l'image hsv

        Args:
            hsv_img (list): image en format hsv

        Returns:
            list: masque de l'image contenant le Rouge
        """
        # define range of red color in HSV
        lower_hsv = np.array([0, 80, 50])
        higher_hsv = np.array([16, 255, 255])

        # generating mask for red color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))

    def mask_bleu(self, hsv_img):
        """Réalise un masque bleu sur l'image hsv

        Args:
            hsv_img (list): image en format hsv

        Returns:
            list: masque de l'image contenant le Bleu
        """
        # define range of blue color in HSV
        lower_hsv = np.array([80, 50, 30])
        higher_hsv = np.array([150, 255, 255])

        # generating mask for blue color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))
    
    
    def create_xyz(self, mask=None):
        """Créer la liste des points (x,y,z) en alignant les depths

        Args:
            mask (list, optional): Masque de couleur utilisé, si un masque est utilisé. Defaults to None.

        Returns:
            list: liste des points (x, y, z)
        """
        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())

        # creation de la liste des points x, y, z en retirant les points de depth 0
        xyz = []
        for x in range(depth_image.shape[0]):
            for y in range(depth_image.shape[1]):
                if mask is None and depth_image[x, y] !=0 :
                    xyz.append([x, y, depth_image[x, y]])
                elif mask is not None and mask[x, y] !=0 and depth_image[x, y] != 0 :
                    xyz.append([x, y, depth_image[x, y]])
                
        self.xyz = np.asanyarray(xyz)
        return self.xyz
    
    def contours(self, mask):
        """détecte les contours des formes du masque

        Args:
            mask (list): masque utilisé

        Returns:
            list: liste des contours trouvés
        """
        elements, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # len(elements) renvoie le nombre de régions de pixels connexes détectés.
        if len(elements) > 0:
            # sorted() permet de trier par ordre décroissant les instances de elements en fonction de leur surface
            c = sorted(elements, key=cv2.contourArea)
            return c

    def centre(self, mask):
        """permet de trouver le centre d'un objet grâce à ses contours

        Args:
            mask (list): masque utilisé

        Returns:
            tuple: pixel correspondant au centre de la forme
        """
        c = self.contours(mask)
        if c is not None:
            # définit le cercle minimum qui couvre complètement l'objet avec une surface minimale
            ((x, y), rayon) = cv2.minEnclosingCircle(c[0])
            return int(x), int(y)
        
    def positionXYZ(self, pixel):
        """Calcule la position en mètres d'un pixel

        Args:
            pixel (tuple ou list): indices X et Y du pixel

        Returns:
            list: point avec coordonnées x, y et z en mètres
        """
        if pixel is None:
            return

        (px, py) = pixel

        depth = self.aligned_depth_frame.get_distance(px, py)
        # print(pixel)
        # X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, pixel, depth)
        point = rs.rs2_deproject_pixel_to_point(self.color_intrinsics, [px, py], depth)
        # en mètres m
        point = [point[0], point[1], point[2]]
        return point
    
    def positions_xyz(self, xyz):
        """Calcul les positions en mètre d'une liste de pixels

        Args:
            xyz (list): image avec depth 

        Returns:
            list: positions 3d en mètres de chaque point
        """
        positions = []
        for i, pixel in enumerate(xyz):
            if pixel[2] != 0:
                positions.append(self.positionXYZ([pixel[1], pixel[0]]))
        return positions


def main():
    """exemple d'utilisation de la classe Camera
    """
    cam = Camera()
    frames, aligned_frames, aligned_depth_frame, color_frame = cam.updateCam()
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    flat_depth=depth_image.flatten()
    # flat_depth=removeOutliers(flat_depth, 2)    
    max_depth=depth_image.max()
    min_depth=depth_image.min()
    depth_image -= min_depth
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=255/(max_depth-min_depth)), cv2.COLORMAP_JET)

    cv2.imshow("Image", depth_colormap)

if __name__=="__main__":
    while(1):
        main()
        time.sleep(0.1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
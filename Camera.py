"""
Class Camera et test cam
"""
import time

# imports
import cv2
import pyrealsense2 as rs
import numpy as np
from filter_outliers import up_down_limits, removeOutliers
import matplotlib.tri as tri


class Camera:

    def __init__(self):
        """
        Utilisation de la camera intel realsense
        """
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
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align.process(self.frames)

        # Get aligned frames
        self.aligned_depth_frame = self.aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        self.color_frame = self.aligned_frames.get_color_frame()
        return self.frames, self.aligned_frames, self.aligned_depth_frame, self.color_frame

    def mask_jaune(self, hsv_img):
        # define range of yellow color in HSV
        lower_hsv = np.array([20, 80, 50])
        higher_hsv = np.array([50, 255, 255])

        # generating mask for blue color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return mask

    def mask_vert(self, hsv_img):
        # define range of blue color in HSV
        lower_hsv = np.array([50, 80, 35])
        higher_hsv = np.array([85, 255, 255])

        # generating mask for blue color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return mask

    def mask_rouge(self, hsv_img):
        # define range of blue color in HSV
        lower_hsv = np.array([0, 80, 50])
        higher_hsv = np.array([16, 255, 255])

        # generating mask for blue color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return mask

    def mask_bleu(self, hsv_img):
        # define range of blue color in HSV
        lower_hsv = np.array([90, 80, 50])
        higher_hsv = np.array([110, 255, 255])

        # generating mask for blue color
        mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
        return mask
    
    def positionXYZ(self, pixel):
        if pixel is None:
            return

        (px, py) = pixel

        depth = self.aligned_depth_frame.get_distance(px, py)
        # X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, pixel, depth)
        point = rs.rs2_deproject_pixel_to_point(self.color_intrinsics, [px, py], depth)
        # en mètres m
        point = [point[0], point[1], point[2]]
        return point
    
    def create_xyz(self):
        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())

        # retirer les valeurs aberrantes
        # flat_depth = depth_image.flatten()
        # borne_inf, borne_sup = up_down_limits(flat_depth, 2)
        # mediane = np.median(flat_depth)

        # valeurs aberrantes = mediane et creation de la liste des points x, y, z
        xyz = []
        for x in range(depth_image.shape[0]):
            for y in range(depth_image.shape[1]):
                # if depth_image[x, y] <=borne_inf or depth_image[x, y]>= borne_sup: #si en dehors des bornes alors valeur aberrante
                #     depth_image[x, y] = mediane
                if depth_image[x, y] !=0 :
                    xyz.append([x, y, depth_image[x, y]])
        self.xyz = np.asanyarray(xyz)
        return self.xyz
    
    def create_triangle(self):
        touslesx = self.xyz[:, 0]
        touslesy = self.xyz[:, 1]
        touslesz = self.xyz[:, 2]

        # creation des triangles
        triangles = tri.Triangulation(touslesx, touslesy)
        return triangles
        


def main():
    cam = Camera()
    frames, aligned_frames, aligned_depth_frame, color_frame = cam.updateCam()
    # frame = np.asanyarray(color_frame.get_data())
    #
    # # conversion hsv
    # frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #
    # # creation des masques
    # rouge = mask_rouge(frame_hsv)
    # bleu = mask_bleu(frame_hsv)
    # jaune = mask_jaune(frame_hsv)
    # vert = mask_vert(frame_hsv)
    #
    # concat = np.hstack([rouge, jaune])
    # concat2 = np.hstack([vert, bleu])
    #
    # # affichage des masques
    # cv2.imshow("mask RJ", concat)
    # cv2.imshow("mask VB", concat2)
    # cv2.imshow("Image", frame)
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    flat_depth=depth_image.flatten()
    flat_depth=removeOutliers(flat_depth, 2)    
    max_depth=flat_depth.max()
    min_depth=flat_depth.min()

    depth_image -= min_depth

    
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=255/(max_depth-min_depth)), cv2.COLORMAP_JET)

    cv2.imshow("Image", depth_colormap)

if __name__=="__main__":
    while(1):
        main()
        time.sleep(0.1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
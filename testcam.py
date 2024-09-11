"""
test voir cam
"""
import time

# imports
import cv2
import pyrealsense2 as rs
import numpy as np

def _initialize_device():
    """
    Utilisation de la camera intel realsense
    """
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Get stream profile and camera intrinsics
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    color_extrinsics = color_profile.get_extrinsics_to(color_profile)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    return pipeline, align, depth_scale, color_intrinsics, color_extrinsics


pipeline, align, depth_scale, color_intrinsics, color_extrinsics = _initialize_device()

def updateCam():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()
    return frames, aligned_frames, aligned_depth_frame, color_frame

def mask_jaune(hsv_img):
    # define range of yellow color in HSV
    lower_hsv = np.array([20, 80, 50])
    higher_hsv = np.array([50, 255, 255])

    # generating mask for blue color
    mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
    return mask

def mask_vert(hsv_img):
    # define range of blue color in HSV
    lower_hsv = np.array([50, 80, 35])
    higher_hsv = np.array([85, 255, 255])

    # generating mask for blue color
    mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
    return mask

def mask_rouge(hsv_img):
    # define range of blue color in HSV
    lower_hsv = np.array([0, 80, 50])
    higher_hsv = np.array([16, 255, 255])

    # generating mask for blue color
    mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
    return mask

def mask_bleu(hsv_img):
    # define range of blue color in HSV
    lower_hsv = np.array([90, 80, 50])
    higher_hsv = np.array([110, 255, 255])

    # generating mask for blue color
    mask = cv2.inRange(hsv_img, lower_hsv, higher_hsv)
    return mask


def main():
    frames, aligned_frames, aligned_depth_frame, color_frame = updateCam()
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
    flat_depth=depth_image.flatten().tolist()
    set_depth = set(flat_depth)
    flat_depth = list(set_depth)
    flat_depth.sort()
    print(flat_depth)
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=255/max_depth), cv2.COLORMAP_JET)

    # cv2.imshow("Image", depth_colormap)

if __name__=="__main__":
    while(1):
        main()
        time.sleep(0.1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
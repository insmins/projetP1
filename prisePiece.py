import time

import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import os
import math
import transformations as tf
import pyrealsense2 as rs
import numpy as np
from Transfo import *
import rtde_receive
import rtde_control
import dashboard_client
from Transfo import *



def initialize_device():
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
    # print(color_intrinsics)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # depth= depth_sensor.get_depth()
    # print("Depth Scale is: " , depth_scale)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    return pipeline, align, depth_scale,color_intrinsics,color_extrinsics


def coordXYZ(x, y, rotation_rx):
    pipeline, align, depth_scale, color_intrinsics,color_extrinsics = initialize_device()

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()
    # Validate that both frames are valid
    # if not aligned_depth_frame or not color_frame:
    depth =aligned_depth_frame.get_distance(x, y)
    # X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, pixel, depth)
    point = rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], depth)
    point=[point[0]*1000, point[1]*1000, point[2]*1000, rotation_rx, 0.0, 0.0]
    return point




"""
Prise de piece
"""

def T_cam2base(T_cam2gripper, T_gripper2base):

    T_cam2base = T_gripper2base @ T_cam2gripper
    return T_cam2base

def pose_piece(T_cam2base_prisDeVue,Coord3D_camera_frame):
    # T_cam2base_prisDeVue=np.array(T_cam2base_prisDeVue)
    # Coord3D_camera_frame=np.array( Coord3D_camera_frame)
    # print('ee',T_cam2base_prisDeVue)
    # print('ff',Coord3D_camera_frame)

    pose_piece = T_cam2base_prisDeVue @ Coord3D_camera_frame
    R, t = matrice_to_Rtvect(T_cam2base_prisDeVue)

    pose_for_robot = [pose_piece[0][0], pose_piece[1][0], pose_piece[2][0], R[0][0], R[1][0], R[2][0]]

    return pose_piece,pose_for_robot

def pose_piece_real(T_cam2base_prisDeVue,pose_piece_in_camera_frame):
    # T_cam2base_prisDeVue=np.array(T_cam2base_prisDeVue)
    # Coord3D_camera_frame=np.array( Coord3D_camera_frame)
    # print('ee',T_cam2base_prisDeVue)
    # print('ff',Coord3D_camera_frame)
    # print(pose_piece_in_camera_frame)
    T_piece2camera= create_matrice(pose_piece_in_camera_frame)

    T_piece2base = T_cam2base_prisDeVue @ T_piece2camera
    print("*********************", T_cam2base_prisDeVue)
    rvec,tvec=matrice_to_rtvect(T_piece2base)
    pose = [tvec[0], tvec[1], tvec[2], np.array(rvec[0])[0], np.array(rvec[1])[0], np.array(rvec[2])[0]]

    # pose_piece = T_cam2base_prisDeVue @ Coord3D_camera_frame
    # R, t = matrice_to_Rtvect(T_cam2base_prisDeVue)
    #
    # pose_for_robot = [pose_piece[0][0], pose_piece[1][0], pose_piece[2][0], R[0][0], R[1][0], R[2][0]]

    return pose


if __name__== "__main__":

    robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
    robot = rtde_control.RTDEControlInterface("10.2.30.60")
    dashboard = dashboard_client.DashboardClient("10.2.30.60")
    joint19=[0.05488887429237366, -1.7198808828936976, 1.8633718490600586, -1.6989052931415003, -1.607802693043844, -1.5686996618853968]
    robot.moveJ(joint19)
    posePrise = robot_r.getActualTCPPose()
    print(posePrise)


    T_base2ee_folder = "Cal2/T_base2ee/"
    T_gripper2base_folder = "Cal2/T_gripper2base/"
    Final_Transform = "FinalTransforms/T_cam2gripper"
    Final_Transform1 = "FinalTransforms/T_gripper2cam"
    Positions_folder = "Cal2/JointPositions/"

    T_base2ee_files = sorted(glob.glob(f'{T_base2ee_folder}/*.npz'))
    T_gripper2base_files = sorted(glob.glob(f'{T_gripper2base_folder}/*.npz'))
    T_cam2gripper_transform_files = sorted(glob.glob(f'{Final_Transform}/*.npz'))
    T_gripper2cam_transform_files = sorted(glob.glob(f'{Final_Transform1}/*.npz'))
    pose_files = sorted(glob.glob(f'{Positions_folder}/*.npz'))

    # Load list of T_cam2gripper gTc
    # T_cam2gripper_transform = [T_cam2gripper_transform_files[i] for i in range(0, 6)]
    All_T_cam2gripper_list = [np.load(f)['arr_0'] for f in T_cam2gripper_transform_files]
    # print(T_cam2gripper_transform_files)





    # Load list of T_gripper2cam cTg
    # T_gripper2cam_transform = [T_cam2gripper_transform_files[i] for i in range(6,11)]
    All_T_gripper2cam_list = [np.load(f)['arr_0'] for f in T_gripper2cam_transform_files]

    T_gripper2base = create_matrice(posePrise)
    T_gripper2base[0, 3] = T_gripper2base[0, 3] * 1000
    T_gripper2base[1, 3] = T_gripper2base[1, 3] * 1000
    T_gripper2base[2, 3] = T_gripper2base[2, 3] * 1000
    # print("******", T_gripper2base)


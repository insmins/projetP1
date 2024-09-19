import cv2
import numpy as np
import glob
import os
import math
# import transformations as tf
import pyrealsense2 as rs
import numpy as np


def pose_to_matrice(pose):
    """
    Calcul d'une matrice à partir d'une pose
    :param pose: pose (x, y, z, rx, ry, rz)
    :return:
    """
    translation = np.array(pose[0:3])  # Translation vector (in m)
    rotation = np.array(pose[3:6])
    Rvect=rotation
    tvect = np.array([[translation[0]], [translation[1]], [translation[2]]]).reshape(3,1)  # Column vector of translation vector (in mm)
    Rvect_3x3 = cv2.Rodrigues(rotation)[0]  # Rodrigues operator for calculting rotation matrix --> scipy converter delivers the same
    # print(t_Mtx)
    Rvect_3x3 = cv2.Rodrigues(Rvect)[0]
    RT = np.concatenate((Rvect_3x3, tvect), axis=1)
    matrice_4x4 = np.concatenate((RT, np.array([[0, 0, 0, 1]])), axis=0)
    return matrice_4x4

def create_matrice(pose):
    """
    Calcul d'une matrice à partir d'une pose
    :param pose: pose (x, y, z, rx, ry, rz)
    :return:
    """
    translation = np.array(pose[0:3])  # Translation vector (in m)
    rotation = np.array(pose[3:6])
    Rvect=rotation
    # # Rotation vector unscaled (in rad)
    # rv_len = math.sqrt(
    #     math.pow(rotation[0], 2) + math.pow(rotation[1], 2) + math.pow(rotation[2], 2))  # Orthogonal Length (in rad)
    # scale = 1 - 2 * math.pi / rv_len  # calculate scaling factor
    # rotation_scaled = scale * rotation  # Rotation vector (in rad)
    # R_Mtx_rodri = cv2.Rodrigues(rotation_scaled)[0]

    tvect = np.array([[translation[0]], [translation[1]], [translation[2]]]).reshape(3,1)  # Column vector of translation vector (in mm)
    Rvect_3x3 = cv2.Rodrigues(rotation)[0]  # Rodrigues operator for calculting rotation matrix --> scipy converter delivers the same
    # print(t_Mtx)
    # Rvect_3x3 = cv2.Rodrigues(Rvect)[0]
    RT = np.concatenate((Rvect_3x3, tvect), axis=1)
    matrice_4x4 = np.concatenate((RT, np.array([[0, 0, 0, 1]])), axis=0)
    return matrice_4x4

def pose_to_Rtvect(pose):
    """
    Calcul de Rvect, tvect à partir d'une pose
    :param pose:
    :return:
    """
    translation = np.array(pose[0:3])  # Translation vector (in m)
    rotation = np.array(pose[3:6])
    Rvect = rotation
    tvect = np.array([[translation[0]], [translation[1]], [translation[2]]]).reshape(3,
                                                                                     1)  # Column vector of translation vector (in mm)
    Rvect_3x3 = cv2.Rodrigues(rotation)[
        0]  # Rodrigues operator for calculting rotation matrix --> scipy converter delivers the same
    # print(t_Mtx)
    Rvect_3x3 = cv2.Rodrigues(Rvect)[0]
    return Rvect_3x3, tvect


def rtvect_to_matrice(rvect,tvect):
    """
    Calcul d'une matrice à partir Rvect,tvect
    :param Rvect:
    :param tvect:
    :return: matrice 4x4
    """

    Rvect_3x3 = cv2.Rodrigues(rvect)[0]
    RT = np.concatenate((Rvect_3x3 , tvect), axis=1)
    matrice_4x4 = np.concatenate((RT, np.array([[0, 0, 0, 1]])), axis=0)

    return matrice_4x4

def Rtvect_to_matrice(Rvec,tvec):
    """
    Calcul d'une matrice à partir Rvect,tvect
    :param Rvect:
    :param tvect:
    :return: matrice 4x4
    """

    RT = np.concatenate((Rvec , tvec), axis=1)
    matrice_4x4 = np.concatenate((RT, np.array([[0, 0, 0, 1]])), axis=0)

    return matrice_4x4
def Rtvect_to_pose(Rvect,tvect):
    """
    calcul de pose à partir de Rvect,tvect
    :param Rvect:
    :param tvect:
    :return: pose (x, y, z, rx, ry, rz)
    """
    pose=[]
    return pose

def matrice_to_pose(matrice):
    """
    calcul de pose à partir d'une matrice 4x4
    :param matrice:
    :return: pose (x, y, z, rx, ry, rz)
    """
    Rotation = matrice[:3, :3]
    Rvect = cv2.Rodrigues(Rotation)[0]
    tvect = matrice[:3, 3]
    pose=[tvect[0],tvect[1],tvect[2],Rvect[0][0],Rvect[1][0],Rvect[2][0] ]
    return pose

def matrice_to_rtvect(matrice):
    """
    Calcul de Rvect, tvect à partir d'une matrice
    :param matrice:
    :return: Rvect, tvect
    """
    Rotation = matrice[:3, :3]
    Rvect  = cv2.Rodrigues(Rotation)[0]
    tvect = matrice[:3, 3]

    return Rvect, tvect

def matrice_to_Rtvect(matrice):
    """
    Calcul de Rvect, tvect à partir d'une matrice
    :param matrice:
    :return: Rvect, tvect
    """
    Rotation = matrice[:3, :3]
    Rvect  = Rotation
    tvect = matrice[:3, 3]

    return Rvect, tvect
def matrice_to_matRotTrans3x3(matrice):
    """
    Calcul de Rvect, tvect à partir d'une matrice
    :param matrice:
    :return: Rvect, tvect
    """
    Rotation3x3 = matrice[:3, :3]
    # Rvect  = [cv2.Rodrigues(Rotation)[0]]
    translation = matrice[:3, 3]

    return Rotation3x3, translation
def inverse_matrice(matrice):
    """
    Calcul l'inverse d'une matrice 4x4
    :param matrice:
    :return: inv_matrice 4x4
    """
    inv_matrice = np.linalg.inv(matrice)
    return inv_matrice

def transformation_pose_A_to_B(matrice_AB,pose):
    """
    Calcul transformation à partir d'une matrice et d'une pose
    :param matrice_AB:
    :param pose: pose (x, y, z, rx, ry, rz)
    :return: pose_A_to_B
    """

    pose_A_to_B=[]

    return pose_A_to_B

def matrice_rotation_3x3(Rvect):
    """
    Vecteur rotation 3x3 en en matrice rotation 3x3
    :param Rvect:
    :return:
    """
    matrice_rotation_3x3 = cv2.Rodrigues(Rvect)[0]
    return matrice_rotation_3x3
def matrice_rotation_4x4(Rvect):
    """
    Vecteur rotation 3x3 en en matrice rotation 3x3
    :param Rvect:
    :return:
    """
    tvect = [0, 0, 0]
    (R, jac) = cv2.Rodrigues(Rvect)  # ignore the jacobian
    M = np.eye(4)
    M[0:3, 0:3] = R
    # M[0:3, 3] = tvect.squeeze()  # 1-D vector, row vector, column vector, whatever
    return M
    # tvect=np.array([0, 0, 0])
    # Rvect_3x3 = cv2.Rodrigues(Rvect)[0]
    # RT = np.concatenate((Rvect_3x3, tvect), axis=1)
    # matrice_rotation_3x3= np.concatenate((RT, np.array([[0, 0, 0, 1]])), axis=0)
    #
    # return matrice_rotation_3x3


def rvect_to_rpy(rvect):
    """
    vecteur rotation en roll, pitch, and yaw
    :param rvect:
    :return:
    """
    rpy=[]
    return  rpy

def pose_to_quaternium(pose,w=1):
    quat = pose[0:4]
    quat[3] = 1
    quat=np.array(quat)
    # quat.reshape(-1, 1)
    return quat.reshape(-1, 1)

if __name__== "__main__":
    tvect = np.array([0, 0, 0, 1]).reshape(4, 1)
    pose = [-0.42689418002561047, -0.10800356900066635, 0.3230311622922157, 0.3385492606897685, 2.8943642251003054,
             0.19777794851259947]
    print(tvect)
    print(pose_to_Rtvect(pose))
    #     v_init = [-0.0012, 3.1162, 0.03889]
    #     v = [-0.06, 0.13, -0.04]
    #
    #
    #     def length(v):
    #         return math.sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2))
    #
    #
    #     def norm(v):
    #         l = length(v)
    #         norm = [v[0] / l, v[1] / l, v[2] / l]
    #         return norm
    #
    #
    #     def _polyscope(rx, ry, rz):
    #         if ((abs(rx) >= 0.001 and rx < 0.0) or (abs(rx) < 0.001 and abs(ry) >= 0.001 and ry < 0.0) or (
    #                 abs(rx) < 0.001 and abs(ry) < 0.001 and rz < 0.0)):
    #
    #             scale = 1 - 2 * math.pi / length([rx, ry, rz])
    #             ret = [scale * rx, scale * ry, scale * rz]
    #             print(scale)
    #             print("PolyScope SCALED value: ", ret)
    #         # return ret
    #         else:
    #             ret = [rx, ry, rz]
    #             print("PolyScope value: ", ret)
    #         return ret
    #
    #
    # def polyscope(v):
    #     return _polyscope(v[0], v[1], v[2])
    #
    #
    # polyscope(v_init)
    # polyscope(v)
    #

"""
Nom du fichier : testgralschmidt.py
Auteur : Mattéo CAUX et Inès EL HADRI
Date : 2024-10-02
Description : Ce script contient des fonctions pour orthogonaliser une base et la rendre directe
"""

#import
import open3d as o3d
import numpy as np
import polyscope as ps

def gramschmit(e1, e2, e3):
    u1 = e1
    u2 = e2 - np.dot(e2, u1)*u1
    u3 = e3 - np.dot(e3, u1)*u1 - np.dot(e3, u2)*u2

    u2 = u2 / np.linalg.norm(u2)
    u3 = u3 / np.linalg.norm(u3)

    return u1, u2, u3

def base_ortho(p1,p2):
     return p1,p2,np.cross(p1,p2)

def directe(u1,u2,u3):
        base_directe=[None]*3

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

        # if np.cross(base_directe[2], base_directe[0])==vects[0]:
        base_directe[1]=vects[0]
        # else :
        #     base_directe[1]= [-x for x in vects[0]]
        if np.linalg.det(base_directe)<0:
            base_directe[1]= [-x for x in base_directe[1]]

        return base_directe

if __name__ == "__main__":
    p1 = [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
    p1 = p1 / np.linalg.norm(p1)
    p2 = [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
    p2 = p2 / np.linalg.norm(p2)
    p3 = [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
    p3 = p3 / np.linalg.norm(p3)

    # p1=np.asanyarray([-1.0,0,0])
    # p2=np.asanyarray([0.0,1,0])
    # p3=np.asanyarray([0.0,0,1])

    # creation du pointcloud open3d
    pcla = o3d.geometry.PointCloud()
    pcla.points = o3d.utility.Vector3dVector(np.asanyarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0], [100, 100, 0.0]]))
    pcla.normals = o3d.utility.Vector3dVector(np.array([p1, p2, p3]))

    u1, u2, u3 = gramschmit(p1, p2, p3)
    # u1, u2, u3 = base_ortho(p1,p2)

    base_directe=directe(u1,u2,u3)

    pclb = o3d.geometry.PointCloud()
    pclb.points = o3d.utility.Vector3dVector(np.array([[0.0, 0, 100], [0.0, 0, 100], [0.0, 0.0, 100]]))
    pclb.normals = o3d.utility.Vector3dVector(np.array([u1, u2, u3]))
    # pclb.normals = o3d.utility.Vector3dVector(np.array([base_directe[0], base_directe[1], base_directe[2]]))

    pclc = o3d.geometry.PointCloud()
    pclc.points = o3d.utility.Vector3dVector(np.array([[0.0, 0, 100], [10.0, 0, 100], [0.0, 100, 10]]))
    pclc.normals = o3d.utility.Vector3dVector(np.array([u1, u2, u3]))
    # pclb.normals = o3d.utility.Vector3dVector(np.array([base_directe[0], base_directe[1], base_directe[2]]))


    o3d.visualization.draw_geometries([pcla, pclb, pclc])

    print(u1,u2,u3)
    print(base_directe)



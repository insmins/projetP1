import open3d as o3d
import numpy as np
from test_cube import tourner, translater, cube, centre, translater_cube, tourner_cube
import polyscope as ps



def gramschmit(e1, e2, e3):
    u1 = e1
    u2 = e2 - np.dot(e2, u1)*u1
    u3 = e3 - np.dot(e3, u1)*u1 - np.dot(e3, u2)*u2

    u2 = u2 / np.linalg.norm(u2)
    u3 = u3 / np.linalg.norm(u3)

    return u1, u2, u3

if __name__ == "__main__":
    p1 = [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
    p1 = p1 / np.linalg.norm(p1)
    p2 = [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
    p2 = p2 / np.linalg.norm(p2)
    p3 = [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
    p3 = p3 / np.linalg.norm(p3)

    # creation du pointcloud open3d
    pcla = o3d.geometry.PointCloud()
    pcla.points = o3d.utility.Vector3dVector(np.asanyarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0], [100, 100, 0.0]]))
    pcla.normals = o3d.utility.Vector3dVector(np.array([p1, p2, p3]))

    u1, u2, u3 = gramschmit(p1, p2, p3)


    pclb = o3d.geometry.PointCloud()
    pclb.points = o3d.utility.Vector3dVector(np.array([[0.0, 0, 100], [0.0, 0, 100], [0.0, 0.0, 100]]))
    pclb.normals = o3d.utility.Vector3dVector(np.array([u1, u2, u3]))

    pclc = o3d.geometry.PointCloud()
    pclc.points = o3d.utility.Vector3dVector(np.array([[0.0, 0, 100], [10.0, 0, 100], [0.0, 100, 10]]))
    pclc.normals = o3d.utility.Vector3dVector(np.array([u1, u2, u3]))


    o3d.visualization.draw_geometries([pcla, pclb, pclc])
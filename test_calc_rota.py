import numpy as np
from Robot import Robot
from Transfo import matrice_to_pose, create_matrice

def rotation(alpha, beta,gamma): 
    #conversion si alpha, beta en degre et non radian 
    alpha=alpha*(np.pi/180)
    beta=beta*(np.pi/180)
    gamma=gamma*(np.pi/180)
    Rx=np.asanyarray([[1,0,0],[0,np.cos(alpha),-np.sin(alpha)],[0,np.sin(alpha),np.cos(alpha)]])
    Ry=np.asanyarray([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]])
    Rz=np.asanyarray([[np.cos(gamma),-np.sin(gamma),0],[np.sin(gamma),np.cos(gamma),0],[0,0,1]])
    return Rx @ Ry @ Rz

def matrice_passage_normale(mat_rot,point):
    res=mat_rot.tolist()
    res.append([0,0,0,1])
    for i in range(len(point)):
        res[i].append(point[i])
    return np.asanyarray(res)


if __name__=="__main__":
    robot = Robot()
    robot.bouger(robot.pos_init, 3, 1)

    point=robot.pos_init[:3]
    alpha=0
    beta=0
    gamma=0
    pos=point+[gamma,np.pi,0]
    robot.bouger(pos,0.5)
    for _ in range(18):
        alpha+=5
        mat4x4=matrice_passage_normale(rotation(gamma,beta,alpha),point)
    #     # print(mat4x4)
        pos=matrice_to_pose(mat4x4)
        pos= robot.correction_pose(pos)
    #     # print(pos)
        print(alpha)
        # pos[5]=gamma*(np.pi/180)
        robot.bouger(pos,0.5)

    # robot.bouger(robot.pos_init, 3, 1)
    # a=create_matrice(robot.pos_init)
    # print(a)
    # print(np.linalg.inv(a))

import numpy as np
from Robot import Robot
from Transfo import matrice_to_pose, create_matrice

def rotation(gamma, beta,alpha): 
    #conversion alpha, beta, gamma radian
    alpha=alpha*(np.pi/180)
    beta=np.pi+beta*(np.pi/180)
    gamma=gamma*(np.pi/180)
    Rx=np.asanyarray([[1,            0,             0],
                      [0,np.cos(gamma),-np.sin(gamma)],
                      [0,np.sin(gamma), np.cos(gamma)]])
    Ry=np.asanyarray([[np.cos(beta) ,0,np.sin(beta)],
                      [0            ,1,           0],
                      [-np.sin(beta),0,np.cos(beta)]])
    Rz=np.asanyarray([[np.cos(alpha),-np.sin(alpha),0],
                      [np.sin(alpha), np.cos(alpha),0],
                      [0            , 0            ,1]])
    return Rz @ Ry @ Rx

def matrice_passage_normale(mat_rot,point):
    res=mat_rot.tolist()
    res.append([0,0,0,1])
    for i in range(len(point)):
        res[i].append(point[i])
    return np.asanyarray(res)


if __name__=="__main__":
    robot = Robot()
    # robot.bouger(robot.pos_init, 3, 1)

    point=robot.pos_init[:3]
    alpha=0 #selon x
    beta=0 # selon y
    gamma=0 # selon z
    # robot.bouger(pos,0.5)
    mat4x4=matrice_passage_normale(rotation(gamma, beta, alpha),point)
    # print("mat4x4 :\n",mat4x4)
    pos=matrice_to_pose(mat4x4)
    # print("pose",pos)
    robot.bouger(pos,0.5)







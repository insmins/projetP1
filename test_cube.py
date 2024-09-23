import polyscope as ps
import numpy as np

len = 2
x = np.asanyarray(np.linspace(0, 0.065, len))
y = x.copy()
z = x.copy()
cube = []


for i in range(len):
    for j in range(len):
        cube.append([x[0], y[i], z[j]])
        cube.append([x[-1], y[i], z[j]])
        cube.append([x[i], y[0], z[j]])
        cube.append([x[i], y[-1], z[j]])
        cube.append([x[i], y[j], z[0]])
        cube.append([x[i], y[j], z[-1]])

cube = np.asanyarray(cube)

centre = np.array([[np.mean(x), np.mean(y), np.mean(z)]])

def tourner(alpha, beta, centre):
    a = centre[0]
    b = centre[1]
    c = centre[2]
    return np.array([
        a*np.cos(alpha)*np.cos(beta) + b*np.sin(alpha)*np.cos(beta) + c*np.sin(beta),
        b*np.cos(alpha) - a*np.sin(alpha),
        - a*np.cos(alpha)*np.sin(beta) - b*np.sin(alpha)*np.sin(beta) + c*np.cos(beta)])
            

def translater(point, centre):
    [x, y, z] = point
    return np.asanyarray([centre[0] + x, centre[1] + y, centre[2] + z])


def tourner_cube(alpha, beta, cube):
    nouveau_cube = cube.copy()
    for p in nouveau_cube:
        a = p[0]
        b = p[1]
        c = p[2]
        p[0] = a*np.cos(alpha)*np.cos(beta) + b*np.sin(alpha)*np.cos(beta) + c*np.sin(beta)
        p[1] = b*np.cos(alpha) - a*np.sin(alpha)
        p[2] = - a*np.cos(alpha)*np.sin(beta) - b*np.sin(alpha)*np.sin(beta) + c*np.cos(beta)
    return nouveau_cube

def translater_cube(point, cube):
    [x, y, z] = point
    return np.asanyarray([[c[0] + x, c[1] + y, c[2] + z] for c in cube])


def main():
    nouveau_cube = translater_cube(np.array([1,1,1]), tourner_cube(1.9, 0.47, cube))
    nouveau_centre = translater(np.array([1,1,1]), tourner(1.9, 0.47, centre[0]))

    ps.init()
    ps.register_point_cloud("centre", nouveau_cube)
    ps.register_point_cloud("my points", np.array([nouveau_centre]))
    ps.show()

if __name__ == "__main__":
    main()
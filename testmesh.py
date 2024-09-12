import pymeshlab
import numpy
import pymeshlab.pmeshlab

# m = pym.MeshSet()
# m.load_new_mesh("C:\\Users\\inese\\Documents\\travail\\CI3\\ProjetP1\\projetp1\\airplane.obj")
# m.save_current_mesh('aiplanee.obj')

verts = numpy.array([
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [-0.5, 0.5, 0.5],
        [0.5, 0.5, 0.5]])

faces = numpy.array([
        [2, 1, 0],
        [1, 2, 3],
        [4, 2, 0],
        [2, 4, 6],
        [1, 4, 0],
        [4, 1, 5],
        [6, 5, 7],
        [5, 6, 4],
        [3, 6, 7],
        [6, 3, 2],
        [5, 3, 7],
        [3, 5, 1]])

pymeshlab
m = pymeshlab.Mesh(verts, faces)
print("fini")

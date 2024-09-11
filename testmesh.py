import pymeshlab as pym
import numpy

verts = numpy.array([
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [-0.5, 0.5, -0.5],
    [0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5],
    [-0.5, 0.5, 0.5],
    [0.5, 0.5, 0.5]])

# create a numpy 12x3 array of faces
# every row represents a face (triangle in this case)
# for every triangle, the index of the vertex
# in the vertex array
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

m = pym.Mesh(verts, faces)
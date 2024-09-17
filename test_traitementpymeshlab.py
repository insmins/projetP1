import pymeshlab as pym

# create a new MeshSet
ms = pym.MeshSet()

# load a new mesh in the MeshSet, and sets it as current mesh
# the path of the mesh can be absolute or relative
ms.load_new_mesh("stl_file\\cam_09161141.stl")
print(ms.mesh_number())
ms.generate_resampled_uniform_mesh()
print(ms.mesh_number())
ms.save_current_mesh("stl_file\\transforme.stl")
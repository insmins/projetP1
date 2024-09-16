from Camera import Camera
import numpy as np
import cv2
from stl import mesh
import datetime

def main():
    cam = Camera()
    frames, aligned_frames, aligned_depth_frame, color_frame = cam.updateCam()
    frame = np.asanyarray(color_frame.get_data())

    # conversion hsv
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # creation des masques
    rouge = cam.mask_rouge(frame_hsv)
    bleu = cam.mask_bleu(frame_hsv)
    jaune = cam.mask_jaune(frame_hsv)
    vert = cam.mask_vert(frame_hsv)

    # # affichage des masques
    # concat = np.hstack([rouge, bleu])
    # concat2 = np.hstack([vert, jaune])
    # cv2.imshow("mask RJ", concat)
    # cv2.imshow("mask VB", concat2)

    # superposition des masques et affichage
    superpo = vert | jaune | rouge | bleu
    # cv2.imshow("superpo", superpo)

    # creation liste xyz
    xyz=cam.create_xyz(mask=superpo)
    # creation des triangles
    triangles =cam.create_triangle()

    # Create the mesh
    cube = mesh.Mesh(np.zeros(triangles.triangles.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(triangles.triangles):
        for j in range(3):
            cube.vectors[i][j] = xyz[f[j],:]

    # Write the mesh to file "cam_MMddhhmm.stl" in the stl_file folder
    t = datetime.datetime.now()
    cube.save('stl_file/cam_'+t.strftime('%m%d%H%M')+'.stl')


    # cv2.imshow("Image", frame)
    # depth_image = np.asanyarray(aligned_depth_frame.get_data())
    # max_depth=depth_image.max()
    # min_depth=depth_image.min()

    # depth_image -= min_depth

    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=255/(max_depth-min_depth)), cv2.COLORMAP_JET)

    # cv2.imshow("Image", depth_colormap)


main()
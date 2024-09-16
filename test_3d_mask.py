from Camera import Camera
import numpy as np
import cv2

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

    # enlever le bruit
    # vert = cv2.morphologyEx(vert, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))
    # jaune = cv2.morphologyEx(jaune, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))
    # rouge = cv2.morphologyEx(rouge, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))
    # bleu = cv2.morphologyEx(bleu, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)))


    # affichage des masques
    concat = np.hstack([rouge, bleu])
    concat2 = np.hstack([vert, jaune])
    cv2.imshow("mask RJ", concat)
    cv2.imshow("mask VB", concat2)

    #superposition des masques et affichage
    # superpo = vert | jaune | rouge | bleu
    # cv2.imshow("mask RJ", superpo)


    cv2.imshow("Image", frame)
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    max_depth=depth_image.max()
    min_depth=depth_image.min()

    depth_image -= min_depth

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=255/(max_depth-min_depth)), cv2.COLORMAP_JET)

    cv2.imshow("Image", depth_colormap)



while 1:
    main()
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
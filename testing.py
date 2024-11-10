import cv2
import numpy as np
import utils

def getLaneCurve(img):

    imgThreshold = utils.thresholding(img)
    cv2.imshow('Thres', imgThreshold)

    return None


if __name__ == "__main__":
    cap = cv2.VideoCapture('./Video/track_vdo_1.mp4')

    while True:
        success, img = cap.read()
        img = cv2.resize(img, (480,240))
        getLaneCurve(img)
        cv2.imshow('Vid', img)
        cv2.waitKey(1)

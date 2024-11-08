import cv2
import numpy as np

def threshold_image(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower_white = np.array([0, 0, 200], dtype=np.uint8)  # Define threshold for white color
    upper_white = np.array([179, 255, 255], dtype=np.uint8)
    maskWhite = cv2.inRange(imgHsv, lower_white, upper_white)
    return maskWhite

def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img

def nothing(a):
    pass

def initializeTrackbars(initialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, nothing)

def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([
        (widthTop, heightTop), 
        (wT - widthTop, heightTop), 
        (widthBottom, heightBottom), 
        (wT - widthBottom, heightBottom)
    ])
    return points

def getLaneCurve(img):
    imgThres = threshold_image(img)
    cv2.imshow('Thresholded', imgThres)

    h, w, c = img.shape
    points = valTrackbars()
    imgWarp = warpImg(img, points, w, h)
    imgWarpPoints = drawPoints(img, points)
    cv2.imshow('Warped Image', imgWarp)
    cv2.imshow('Warp Points', imgWarpPoints)
    return

def warpImg(img, points, w, h):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

if __name__ == '__main__':
    # cap = cv2.VideoCapture('./test_videos/solidWhiteRight.mp4')
    cap = cv2.VideoCapture('./Video/track_vdo_2.mp4')
    initialTracbarVals = [100, 100, 100, 240]
    initializeTrackbars(initialTracbarVals)
    
    while cap.isOpened():
        success, img = cap.read()
        if not success:
            break

        img = cv2.resize(img, (480, 240))
        getLaneCurve(img)
        cv2.imshow('Video', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

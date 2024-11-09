# import cv2
# import numpy as np
 
# def threshold_image(img):
#     imgHsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
#     # lower_white = np.array([130, 129, 127], dtype=np.uint8)  # Define threshold for white color
#     # upper_white = np.array([110, 109, 107], dtype=np.uint8)
#     lower_white = np.array([0, 0, 200], dtype=np.uint8)  # Define threshold for white color
#     upper_white = np.array([179, 255, 255], dtype=np.uint8)
#     maskWhite = cv2.inRange(imgHsv, lower_white, upper_white)
#     return maskWhite
 
# def drawPoints(img, points):
#     for x in range(4):
#         cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
#     return img
 
# def nothing(a):
#     pass
 
# def initializeTrackbars(initialTracbarVals, wT=480, hT=240):
#     cv2.namedWindow("Trackbars")
#     cv2.resizeWindow("Trackbars", 360, 240)
#     cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, nothing)
#     cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, nothing)
#     cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, nothing)
#     cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, nothing)
 
# def valTrackbars(wT=480, hT=240):
#     widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
#     heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
#     widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
#     heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
#     points = np.float32([
#         (widthTop, heightTop),
#         (wT - widthTop, heightTop),
#         (widthBottom, heightBottom),
#         (wT - widthBottom, heightBottom)
#     ])
#     return points
 
# def getLaneCurve(img):
#     imgThres = threshold_image(img)
#     cv2.imshow('Thresholded', imgThres)
 
#     h, w, c = img.shape
#     points = valTrackbars()
#     imgWarp = warpImg(img, points, w, h)
#     imgWarpPoints = drawPoints(img, points)
#     cv2.imshow('Warped Image', imgWarp)
#     cv2.imshow('Warp Points', imgWarpPoints)
#     return
 
# def warpImg(img, points, w, h):
#     pts1 = np.float32(points)
#     pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
#     matrix = cv2.getPerspectiveTransform(pts1, pts2)
#     imgWarp = cv2.warpPerspective(img, matrix, (w, h))
#     return imgWarp
 
# if __name__ == '__main__':
#     cap = cv2.VideoCapture('./test_videos/test1.mp4')
#     # cap = cv2.VideoCapture('./Video/track_vdo_1.mp4')
#     initialTracbarVals = [100, 80, 20, 200]
#     initializeTrackbars(initialTracbarVals)
   
#     while cap.isOpened():
#         success, img = cap.read()
#         if not success:
#             break
 
#         img = cv2.resize(img, (480, 240))
#         getLaneCurve(img)
#         cv2.imshow('Video', img)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
 
#     cap.release()
#     cv2.destroyAllWindows()
import cv2
import numpy as np
 
# Step 1: Color threshold to detect lanes on a dark road
def threshold_image(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Adjust HSV ranges for lane detection on black roads
    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([179, 80, 255], dtype=np.uint8)
    maskWhite = cv2.inRange(imgHsv, lower_white, upper_white)
    return maskWhite
 
# Step 2: Define a region of interest (ROI) to focus on lanes
def region_of_interest(img, points):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
 
# Step 3: Draw points for visualization
def draw_points(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 10, (0, 255, 0), cv2.FILLED)
    return img
 
def initialize_trackbars(initialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, lambda x: None)
    cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, lambda x: None)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, lambda x: None)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, lambda x: None)
 
# Step 4: Read values from trackbars
def val_trackbars(wT=480, hT=240):
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
 
# Step 5: Warp image to get a bird’s-eye view
def warp_image(img, points, w, h):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp
 
def getHistogram(img,minPer=0.1,display=False):
    histValues=np.sum(img,axis=0)
    maxValue=np.max(histValues)
    print(maxValue)
    minValue=minPer*maxValue
    print(minValue)
 
    indexArray=np.where(histValues>=minValue)
    basePoint=int(np.average(indexArray))
    print(basePoint)
    print(histValues)
    if(display):
        imgHist=np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x,intensity in enumerate(histValues):
            cv2.line(imgHist,(x,img.shape[0]),(x,intensity//255),(255,0,255),1)
        return basePoint,imgHist
    return basePoint
   
 
# Main function to preprocess and detect lanes
def get_lane_curve(img):
    imgThres = threshold_image(img)
    cv2.imshow('Thresholded Image', imgThres)  # Show thresholded image
 
    # Get region of interest
    h, w, c = img.shape
    points = val_trackbars()
    imgROI = region_of_interest(imgThres, points)
    cv2.imshow('Region of Interest', imgROI)
 
    # Apply perspective warp
    imgWarp = warp_image(imgROI, points, w, h)
    imgWarpPoints = draw_points(img, points)
    cv2.imshow('Warped Image', imgWarp)
    cv2.imshow('Warp Points', imgWarpPoints)
 
    ##step 3
    basePoints,imgHist=getHistogram(imgWarp,display=True)
    cv2.imshow('Thres',imgThres)
    cv2.imshow('Warp',imgWarp)
    cv2.imshow('Warp Points',imgWarpPoints)
    cv2.imshow('Histogram',imgHist)
    return imgWarp
 
if __name__ == '__main__':
    cap = cv2.VideoCapture('./test_videos/vdo_41sec.mp4')  # Ensure path is correct
    initialTracbarVals = [100, 80, 20, 200]
    initialize_trackbars(initialTracbarVals)
   
    while cap.isOpened():
        success, img = cap.read()
        if not success:
            break
 
        img = cv2.resize(img, (480, 240))
        lane_image = get_lane_curve(img)
        cv2.imshow('Processed Video', lane_image)
 
        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    cap.release()
    cv2.destroyAllWindows()
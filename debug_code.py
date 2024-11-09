import cv2
import numpy as np
# Step 1: Color threshold to detect black or grey regions on the road
def threshold_image(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   
    # Define HSV ranges for black and grey color detection
    # Black color: low brightness (V) and low saturation (S)
    lower_black = np.array([0, 0, 0], dtype=np.uint8)  # Low S and low V for black
    upper_black = np.array([179, 255, 50], dtype=np.uint8)  # Low V for black
   
    # Grey color: low to medium saturation (S), medium brightness (V)
    lower_grey = np.array([0, 0, 50], dtype=np.uint8)  # Grey (low saturation, medium brightness)
    upper_grey = np.array([179, 50, 200], dtype=np.uint8)  # Allow grey shades
   
    # Combine both black and grey masks
    mask_black = cv2.inRange(imgHsv, lower_black, upper_black)
    mask_grey = cv2.inRange(imgHsv, lower_grey, upper_grey)
   
    # Combine both masks for black and grey detection
    mask = cv2.bitwise_or(mask_black, mask_grey)
    return mask
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
def warp_image(img, points, w, h, inv = False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp
def getHistogram(img, minPer=0.2, display=False, region =1):
    if region ==1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)
 
    #print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer * maxValue
 
    indexArray = np.where(histValues <= minValue)
    basePoint = int(np.average(indexArray))
   
    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255//region), (37, 150, 10), 1)
            cv2.circle(imgHist, (basePoint, img.shape[0]), 20,(0, 255, 255),cv2.FILLED)
        return basePoint, imgHist
    return basePoint
# Main function to preprocess and detect lanes
curveList =[]
avgValue =10
 
def get_lane_curve(img, display =2):
    imgCopy = img.copy()
    imgResults = img.copy()
    imgThres = threshold_image(img)
    cv2.imshow('Thresholded Image', imgThres)  # Show thresholded image
   
    # Get region of interest
    hT, wT, c = img.shape
    points = val_trackbars()
    imgROI = region_of_interest(imgThres, points)
    cv2.imshow('Region of Interest', imgROI)
   
    # Apply perspective warp
    imgWarp = warp_image(imgROI, points, wT, hT)
    imgWarpPoints = draw_points(img, points)
    cv2.imshow('Warped Image', imgWarp)
    cv2.imshow('Warp Points', imgWarpPoints)
   
    ## Step 3: Show histogram
    middlePoint, imgHist = getHistogram(imgWarp, display=True,minPer=0.5, region=4)
    curveAveragePoint, imgHist = getHistogram(imgWarp, display=True,minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint # these will be curve vavlues
    return imgWarp
# def stackImages(scale,imgArray):
#     rows = len(imgArray)
#     cols= len(imgArray[0])
#     rowsAvailable = isinstance(imgArray[0], list)
#     width= imgArray[0][0].shape[1]
#     height = imgArray[0][0].shape[0]
#     if rowsAvailable:
#         for x in range_(0, rows):
#             for y in range(0, cols):
#                 if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
#                     imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
#                 else:
#                     imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], img #
#                 if len(imgArray[x][y].shape) == 2; imgArray[x][y]= cv2.cvtColor(_imgArray[x][ #
#         imageBlank = np.zeros((height, width, 3), np.uint8)
#         hor = [imageBlank]*rows
#         hor_con = [imageBlank]*rows
#         for x in range(0, rows):
#             hor[x] = np.hstack(imgArray[x])
#         ver = np.vstack(hor)
#     else:
#         for x in range(0,rows):
#             if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
#                 imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
#             else:
#                 imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape #
#             if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR #
#         hor= np.hstack(imgArray)
#         ver = hor
#     return ver
 
def stackImages(scale, imgArray):
    # Get the number of rows and columns in the image array
    rows = len(imgArray)
    cols = len(imgArray[0])
 
    # Check if the images are nested (2D list) or a single row (1D list)
    rowsAvailable = isinstance(imgArray[0], list)
 
    # Get the width and height of the first image in the array (assumes all images are the same size)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
 
    # Check if the images are in a 2D list (multiple rows and columns)
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # If the image size is not the same as the first one, resize it
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]))
               
                # If the image is grayscale, convert it to RGB
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
 
        # Create a blank image to stack horizontally (initializing the "rows" list with blank images)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
 
        # Stack the images horizontally row by row
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
 
        # Stack the rows vertically
        ver = np.vstack(hor)
   
    # If the images are in a single row (1D list)
    else:
        for x in range(0, rows):
            # Resize images if needed to match the first image in the array
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]))
           
            # If the image is grayscale, convert it to RGB
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
 
        # Stack the images horizontally for a single row
        hor = np.hstack(imgArray)
 
        # The vertical stack is the same as the horizontal stack for a single row
        ver = hor
 
    return ver
 
 
 
 
 
##step 4 - averaging
    curveList.append(curveRaw)
    if len(curveList)>avgVal:
        curveList.pop(0)
    curve = int(sum(curveList)/len(curveList))
 
## step 5 display
    # if display !=0:
    #     imgInvWarp = warping(iMgWarp, points, wT, hT, Iinv=True)
    #     imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
    #     imgInvWarp[0:hT // 3, 0:wT]= 0, 0, 0
    #     imgLaneColor = np.zeros_like(img)
    #     imgLaneColor[:] = 0, 255, 0
    #     imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
    #     imgResult = cv2.addWeighted (imgResult, 1, imgLaneColor, 1, 0)
    #     midy 450
    #     cv2.putText(imgResult, str(curve), (wT // 2-80, 85) cv2.FONT_HERSHEY_COMPLEX, 2, (255,0,0))
    #     cv2.line(imgResult, (WT//2, midY), (wT//2+(curve*3), midY), (255,0,255), 5)
    #     cv2.line(imgResult, ((WT//2 + (curve*3)), midY-25), (wT // 2 +(curve* 3), midY)
    #     for x in range(-30, 30):
    #         w = wT // 20
    #         cv2.line(ingResult, (w*x + int(curve // 50), midY - 10),
    #                 (w*x int(curve // 55), midy + 10), (0, 0, 255), 2)
    #     # fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    #     # cv2.putText(imgResult, FPS+ str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
 
    # if display as 2:
    #     imgStacked = stackImages(0.7, ([img, imgWarpPoints, imgWarp],
    #                                     [mgHist, imgLaneColor, imgResult]))
 
    #     cv2.imshow('ImageStack', imgStacked)
 
    # elif display 1:
    #     cv2.imshow('Result', imgResult)
 
    if display != 0:
        # Inverse warp the image to get the original view
        imgInvWarp = warping(imgWarp, points, wT, hT, Iinv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
 
        # Remove the top part of the image to make space for text or other visuals
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
 
        # Create a blank image for lane coloring and apply the bitwise and to isolate lanes
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0  # Green color for lanes
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
 
        # Add the lane coloring to the result image
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
 
        # Mid Y position for lane curve display
        midY = 450  # Fixed value for vertical position of lane information
 
        # Display the curve value at the top of the image
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 0), 2)
 
        # Draw the lane curve on the result image (a line indicating the curvature)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
 
        # Draw lane markers
        for x in range(-30, 30):  # Adjust range for desired marker placement
            w = wT // 20
            # Line positions depend on the curve and range
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 55), midY + 10), (0, 0, 255), 2)
 
        # Optionally, display FPS if needed
        # fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        # cv2.putText(imgResult, "FPS: " + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
 
    if display == 2:
        # Stack multiple images for visualization
        imgStacked = stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                      [imgHist, imgLaneColor, imgResult]))
 
        # Show stacked images
        cv2.imshow('ImageStack', imgStacked)
 
    elif display == 1:
        # Show the result image with lane detections
        cv2.imshow('Result', imgResult)
 
 
 
 
    cv2.imshow('Histogram', imgHist)
    return imgWarp
if __name__ == '__main__':
    cap = cv2.VideoCapture('./video/track_vdo_1.mp4')  # Ensure path is correct
    initialTracbarVals = [162, 103, 33, 226]
    initialize_trackbars(initialTracbarVals)
    # frameCounter = 0
    curveList = []
   
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
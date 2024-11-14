# import cv2
# import numpy as np
# # Step 1: Color threshold to detect black or grey regions on the road
# def threshold_image(img):
#     imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
#     # Define HSV ranges for black and grey color detection
#     # Black color: low brightness (V) and low saturation (S)
#     lower_black = np.array([0, 0, 0], dtype=np.uint8)  # Low S and low V for black
#     upper_black = np.array([179, 255, 50], dtype=np.uint8)  # Low V for black
    
#     # Grey color: low to medium saturation (S), medium brightness (V)
#     lower_grey = np.array([0, 0, 50], dtype=np.uint8)  # Grey (low saturation, medium brightness)
#     upper_grey = np.array([179, 50, 200], dtype=np.uint8)  # Allow grey shades
    
#     # Combine both black and grey masks
#     mask_black = cv2.inRange(imgHsv, lower_black, upper_black)
#     mask_grey = cv2.inRange(imgHsv, lower_grey, upper_grey)
    
#     # Combine both masks for black and grey detection
#     mask = cv2.bitwise_or(mask_black, mask_grey)
#     return mask
# # Step 2: Define a region of interest (ROI) to focus on lanes
# def region_of_interest(img, points):
#     mask = np.zeros_like(img)
#     cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
#     masked_image = cv2.bitwise_and(img, mask)
#     return masked_image
# # Step 3: Draw points for visualization
# def draw_points(img, points):
#     for x in range(4):
#         cv2.circle(img, (int(points[x][0]), int(points[x][1])), 10, (0, 255, 0), cv2.FILLED)
#     return img
# def initialize_trackbars(initialTracbarVals, wT=480, hT=240):
#     cv2.namedWindow("Trackbars")
#     cv2.resizeWindow("Trackbars", 360, 240)
#     cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, lambda x: None)
#     cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, lambda x: None)
#     cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, lambda x: None)
#     cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, lambda x: None)
# # Step 4: Read values from trackbars
# def val_trackbars(wT=480, hT=240):
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
# # Step 5: Warp image to get a bird’s-eye view
# def warp_image(img, points, w, h):
#     pts1 = np.float32(points)
#     pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
#     matrix = cv2.getPerspectiveTransform(pts1, pts2)
#     imgWarp = cv2.warpPerspective(img, matrix, (w, h))
#     return imgWarp
# def getHistogram(img, minPer=0.2, display=False):
#     histValues = np.sum(img, axis=0)
#     maxValue = np.max(histValues)
#     minValue = minPer * maxValue
#     indexArray = np.where(histValues <= minValue)
#     basePoint = int(np.average(indexArray))
    
#     if display:
#         imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
#         for x, intensity in enumerate(histValues):
#             cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255), (37, 150, 190), 1)
#         return basePoint, imgHist
#     return basePoint
# # Main function to preprocess and detect lanes
# def get_lane_curve(img):
#     imgThres = threshold_image(img)
#     cv2.imshow('Thresholded Image', imgThres)  # Show thresholded image
    
#     # Get region of interest
#     h, w, c = img.shape
#     points = val_trackbars()
#     imgROI = region_of_interest(imgThres, points)
#     cv2.imshow('Region of Interest', imgROI)
    
#     # Apply perspective warp
#     imgWarp = warp_image(imgROI, points, w, h)
#     imgWarpPoints = draw_points(img, points)
#     cv2.imshow('Warped Image', imgWarp)
#     cv2.imshow('Warp Points', imgWarpPoints)
    
#     ## Step 3: Show histogram
#     basePoints, imgHist = getHistogram(imgWarp, display=True)
#     cv2.imshow('Histogram', imgHist)
#     return imgWarp
# if __name__ == '__main__':
#     cap = cv2.VideoCapture('./video/track_vdo_1.mp4')  # Ensure path is correct
#     initialTracbarVals = [162, 103, 33, 226]
#     initialize_trackbars(initialTracbarVals)
    
#     while cap.isOpened():
#         success, img = cap.read()
#         if not success:
#             break
        
#         img = cv2.resize(img, (480, 240))
#         lane_image = get_lane_curve(img)
#         cv2.imshow('Processed Video', lane_image)
        
#         # Press 'q' to exit the loop
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
    
#     cap.release()
#     cv2.destroyAllWindows()


# lanedetection perfectly working and servo integrsated with that on teh rpi 



# import cv2
# import numpy as np
# # Step 1: Color threshold to detect black or grey regions on the road
# def threshold_image(img):
#     imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
#     # Define HSV ranges for black and grey color detection
#     # Black color: low brightness (V) and low saturation (S)
#     lower_black = np.array([0, 0, 0], dtype=np.uint8)  # Low S and low V for black
#     upper_black = np.array([179, 255, 50], dtype=np.uint8)  # Low V for black
    
#     # Grey color: low to medium saturation (S), medium brightness (V)
#     lower_grey = np.array([0, 0, 50], dtype=np.uint8)  # Grey (low saturation, medium brightness)
#     upper_grey = np.array([179, 50, 200], dtype=np.uint8)  # Allow grey shades
    
#     # Combine both black and grey masks
#     mask_black = cv2.inRange(imgHsv, lower_black, upper_black)
#     mask_grey = cv2.inRange(imgHsv, lower_grey, upper_grey)
    
#     # Combine both masks for black and grey detection
#     mask = cv2.bitwise_or(mask_black, mask_grey)
#     return mask
# # Step 2: Define a region of interest (ROI) to focus on lanes
# def region_of_interest(img, points):
#     mask = np.zeros_like(img)
#     cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
#     masked_image = cv2.bitwise_and(img, mask)
#     return masked_image
# # Step 3: Draw points for visualization
# def draw_points(img, points):
#     for x in range(4):
#         cv2.circle(img, (int(points[x][0]), int(points[x][1])), 10, (0, 255, 0), cv2.FILLED)
#     return img
# def initialize_trackbars(initialTracbarVals, wT=480, hT=240):
#     cv2.namedWindow("Trackbars")
#     cv2.resizeWindow("Trackbars", 360, 240)
#     cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, lambda x: None)
#     cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, lambda x: None)
#     cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, lambda x: None)
#     cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, lambda x: None)
# # Step 4: Read values from trackbars
# def val_trackbars(wT=480, hT=240):
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
# # Step 5: Warp image to get a bird’s-eye view
# def warp_image(img, points, w, h):
#     pts1 = np.float32(points)
#     pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
#     matrix = cv2.getPerspectiveTransform(pts1, pts2)
#     imgWarp = cv2.warpPerspective(img, matrix, (w, h))
#     return imgWarp
# def getHistogram(img, minPer=0.2, display=False):
#     histValues = np.sum(img, axis=0)
#     maxValue = np.max(histValues)
#     minValue = minPer * maxValue
#     indexArray = np.where(histValues <= minValue)
#     basePoint = int(np.average(indexArray))
    
#     if display:
#         imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
#         for x, intensity in enumerate(histValues):
#             cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255), (37, 150, 190), 1)
#         return basePoint, imgHist
#     return basePoint
# # Main function to preprocess and detect lanes
# def get_lane_curve(img):
#     imgThres = threshold_image(img)
#     cv2.imshow('Thresholded Image', imgThres)  # Show thresholded image
    
#     # Get region of interest
#     h, w, c = img.shape
#     points = val_trackbars()
#     imgROI = region_of_interest(imgThres, points)
#     cv2.imshow('Region of Interest', imgROI)
    
#     # Apply perspective warp
#     imgWarp = warp_image(imgROI, points, w, h)
#     imgWarpPoints = draw_points(img, points)
#     cv2.imshow('Warped Image', imgWarp)
#     cv2.imshow('Warp Points', imgWarpPoints)
    
#     ## Step 3: Show histogram
#     basePoints, imgHist = getHistogram(imgWarp, display=True)
#     cv2.imshow('Histogram', imgHist)
#     return imgWarp
# if __name__ == '__main__':
#     cap = cv2.VideoCapture('./video/track_vdo_1.mp4')  # Ensure path is correct
#     initialTracbarVals = [162, 103, 33, 226]
#     initialize_trackbars(initialTracbarVals)
    
#     while cap.isOpened():
#         success, img = cap.read()
#         if not success:
#             break
        
#         img = cv2.resize(img, (480, 240))
#         lane_image = get_lane_curve(img)
#         cv2.imshow('Processed Video', lane_image)
        
#         # Press 'q' to exit the loop
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
    
#     cap.release()
#     cv2.destroyAllWindows()



# ***********************************RED LINE IS BEING PRINTED IN TEH MIDDDLE OF THE HISTOGRAM ************
# import cv2
# import numpy as np

# # Step 1: Color threshold to detect black or grey regions on the road
# def threshold_image(img):
#     imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
#     # Define HSV ranges for black and grey color detection
#     # Black color: low brightness (V) and low saturation (S)
#     lower_black = np.array([0, 0, 0], dtype=np.uint8)  # Low S and low V for black
#     upper_black = np.array([179, 255, 50], dtype=np.uint8)  # Low V for black
    
#     # Grey color: low to medium saturation (S), medium brightness (V)
#     lower_grey = np.array([0, 0, 50], dtype=np.uint8)  # Grey (low saturation, medium brightness)
#     upper_grey = np.array([179, 50, 200], dtype=np.uint8)  # Allow grey shades
    
#     # Combine both black and grey masks
#     mask_black = cv2.inRange(imgHsv, lower_black, upper_black)
#     mask_grey = cv2.inRange(imgHsv, lower_grey, upper_grey)
    
#     # Combine both masks for black and grey detection
#     mask = cv2.bitwise_or(mask_black, mask_grey)
#     return mask

# # Step 2: Define a region of interest (ROI) to focus on lanes
# def region_of_interest(img, points):
#     mask = np.zeros_like(img)
#     cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
#     masked_image = cv2.bitwise_and(img, mask)
#     return masked_image

# # Step 3: Draw points for visualization
# def draw_points(img, points):
#     for x in range(4):
#         cv2.circle(img, (int(points[x][0]), int(points[x][1])), 10, (0, 255, 0), cv2.FILLED)
#     return img

# # Trackbar initialization
# def initialize_trackbars(initialTracbarVals, wT=480, hT=240):
#     cv2.namedWindow("Trackbars")
#     cv2.resizeWindow("Trackbars", 360, 240)
#     cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, lambda x: None)
#     cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, lambda x: None)
#     cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, lambda x: None)
#     cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, lambda x: None)

# # Read values from trackbars
# def val_trackbars(wT=480, hT=240):
#     widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
#     heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
#     widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
#     heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
#     points = np.float32([  # The coordinates for the region of interest
#         (widthTop, heightTop),
#         (wT - widthTop, heightTop),
#         (widthBottom, heightBottom),
#         (wT - widthBottom, heightBottom)
#     ])
#     return points

# # Step 5: Warp image to get a bird’s-eye view
# def warp_image(img, points, w, h):
#     pts1 = np.float32(points)
#     pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
#     matrix = cv2.getPerspectiveTransform(pts1, pts2)
#     imgWarp = cv2.warpPerspective(img, matrix, (w, h))
#     return imgWarp

# # Get histogram for base points
# def getHistogram(img, minPer=0.2, display=False):
#     histValues = np.sum(img, axis=0)  # Sum pixel values along columns
#     maxValue = np.max(histValues)
#     minValue = minPer * maxValue
#     indexArray = np.where(histValues <= minValue)
#     basePoint = int(np.average(indexArray))  # Calculate base point as average of indices where values are lower than minValue
    
#     if display:
#         imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
#         for x, intensity in enumerate(histValues):
#             cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255), (37, 150, 190), 1)
#             middle_x = img.shape[1] // 2  # Middle of the histogram width
#             cv2.line(imgHist, (middle_x, 0), (middle_x, img.shape[0]), (0, 0, 255), 2)  # Red line

#         return basePoint, imgHist
#     return basePoint

# # Main function to preprocess and detect lanes
# def get_lane_curve(img):
#     imgThres = threshold_image(img)
#     cv2.imshow('Thresholded Image', imgThres)  # Show thresholded image
    
#     # Get region of interest
#     h, w, c = img.shape
#     points = val_trackbars()
#     imgROI = region_of_interest(imgThres, points)
#     cv2.imshow('Region of Interest', imgROI)
    
#     # Apply perspective warp
#     imgWarp = warp_image(imgROI, points, w, h)
#     imgWarpPoints = draw_points(img, points)
#     cv2.imshow('Warped Image', imgWarp)
#     cv2.imshow('Warp Points', imgWarpPoints)
    
#     ## Step 3: Show histogram
#     basePoints, imgHist = getHistogram(imgWarp, display=True)
#     cv2.imshow('Histogram', imgHist)

#     # Display the middle line in red color
#     middleY = img.shape[0] // 2
#     cv2.line(imgWarp, (basePoints, middleY), (basePoints, img.shape[0]), (0, 0, 255), 3)  # Red color for middle line
    
#     return imgWarp

# if __name__ == '__main__':
#     cap = cv2.VideoCapture('./video/track_vdo_1.mp4')  # Ensure path is correct
#     initialTracbarVals = [162, 103, 33, 226]
#     initialize_trackbars(initialTracbarVals)
    
#     while cap.isOpened():
#         success, img = cap.read()
#         if not success:
#             break
        
#         img = cv2.resize(img, (480, 240))
#         lane_image = get_lane_curve(img)
#         cv2.imshow('Processed Video', lane_image)
        
#         # Press 'q' to exit the loop
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
    
#     cap.release()
#     cv2.destroyAllWindows()
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up GPIO for servo control
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)  # Pin 11 for servo1, pulse 50Hz

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)

# Function to smoothly set servo to the specified angle
def set_servo_angle(angle):
    duty_cycle = 2 + (angle / 18)  # Map angle 0-180 to duty cycle 2-12
    servo1.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Wait for servo to reach position
    servo1.ChangeDutyCycle(0)  # Turn off pulse to hold position

# Step 1: Color threshold to detect black or grey regions on the road
def threshold_image(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_black = np.array([0, 0, 0], dtype=np.uint8)  # Low S and low V for black
    upper_black = np.array([179, 255, 50], dtype=np.uint8)  # Low V for black
    
    lower_grey = np.array([0, 0, 50], dtype=np.uint8)  # Grey (low saturation, medium brightness)
    upper_grey = np.array([179, 50, 200], dtype=np.uint8)  # Allow grey shades
    
    mask_black = cv2.inRange(imgHsv, lower_black, upper_black)
    mask_grey = cv2.inRange(imgHsv, lower_grey, upper_grey)
    
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

# Trackbar initialization
def initialize_trackbars(initialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, lambda x: None)
    cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, lambda x: None)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, lambda x: None)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, lambda x: None)

# Read values from trackbars
def val_trackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([  # The coordinates for the region of interest
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

# Get histogram for base points
def getHistogram(img, minPer=0.2, display=False):
    histValues = np.sum(img, axis=0)  # Sum pixel values along columns
    maxValue = np.max(histValues)
    minValue = minPer * maxValue
    indexArray = np.where(histValues <= minValue)
    basePoint = int(np.average(indexArray))  # Calculate base point as average of indices where values are lower than minValue
    
    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255), (37, 150, 190), 1)
            middle_x = img.shape[1] // 2  # Middle of the histogram width
            cv2.line(imgHist, (middle_x, 0), (middle_x, img.shape[0]), (0, 0, 255), 2)  # Red line

        return basePoint, imgHist
    return basePoint

# Main function to preprocess and detect lanes
def get_lane_curve(img):
    imgThres = threshold_image(img)
    # cv2.imshow('Thresholded Image', imgThres)  # Show thresholded image---
    
    # Get region of interest
    h, w, c = img.shape
    points = val_trackbars()
    imgROI = region_of_interest(imgThres, points)
    cv2.imshow('Region of Interest', imgROI)
    
    # Apply perspective warp
    imgWarp = warp_image(imgROI, points, w, h)
    imgWarpPoints = draw_points(img, points)
    # cv2.imshow('Warped Image', imgWarp)
    cv2.imshow('Warp Points', imgWarpPoints)
    
    ## Step 3: Show histogram
    basePoints, imgHist = getHistogram(imgWarp, display=True)
    cv2.imshow('Histogram', imgHist)

    # Get the middle of the frame and histogram
    middle_frame = img.shape[1] // 2  # Middle point of the frame (x-coordinate)
    middle_histogram = basePoints  # Middle of the histogram is represented by the basePoint

    # Calculate the difference between the middle of the frame and the middle of the histogram
    difference = middle_frame - middle_histogram
    turnDirection = ""


     # Visualize turn direction based on the difference
    if abs(difference) < 60:
        turnDirection = "Straight"
        # Visualize straight direction
        cv2.putText(imgHist, turnDirection, (w // 2 - 80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
        print("Go Straight")
        set_servo_angle(24)  # Set servo to 24 degrees for straight
    elif difference > 60:
        turnDirection = "Turn Left"
        # Visualize turn direction
        cv2.putText(imgHist, turnDirection, (w // 2 - 80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
        print("Turn Left")
        set_servo_angle(1)  # Set servo to 1 degree for left turn
    elif difference < -60:
        turnDirection = "Turn Right"
        # Visualize turn direction
        cv2.putText(imgHist, turnDirection, (w // 2 - 80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        print("Turn Right")
        set_servo_angle(45)  # Set servo to 45 degrees for right turn

    return imgHist

if __name__ == '__main__':
    cap = cv2.VideoCapture('./video/track_vdo_1.mp4')  # Ensure path is correct
    initialTracbarVals = [162, 103, 33, 226]
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

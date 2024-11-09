import cv2
import numpy as np

# Step 1: Color threshold to detect black or grey regions on the road
def threshold_image(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Define HSV ranges for black and grey color detection
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([179, 255, 50], dtype=np.uint8)
    lower_grey = np.array([0, 0, 50], dtype=np.uint8)
    upper_grey = np.array([179, 50, 200], dtype=np.uint8)
    
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

# Step 6: Calculate histogram to detect the lane center
def getHistogram(img, minPer=0.2, display=False):
    histValues = np.sum(img, axis=0)
    maxValue = np.max(histValues)
    minValue = minPer * maxValue
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    
    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255), (37, 150, 190), 1)
        return basePoint, imgHist
    return basePoint

# Step 7: Calculate steering angle
def calculate_steering_angle(basePoint, imgWidth):
    lane_center = basePoint
    frame_center = imgWidth // 2
    deviation = frame_center - lane_center
    steering_angle = np.arctan2(deviation, imgWidth) * (180 / np.pi)  # Convert to degrees
    return steering_angle

# Main function to preprocess and detect lanes
def get_lane_curve(img):
    imgThres = threshold_image(img)
    cv2.imshow('Thresholded Image', imgThres)
    
    h, w, c = img.shape
    points = val_trackbars()
    imgROI = region_of_interest(imgThres, points)
    cv2.imshow('Region of Interest', imgROI)
    
    imgWarp = warp_image(imgROI, points, w, h)
    imgWarpPoints = draw_points(img, points)
    cv2.imshow('Warped Image', imgWarp)
    cv2.imshow('Warp Points', imgWarpPoints)
    
    basePoint, imgHist = getHistogram(imgWarp, display=True)
    cv2.imshow('Histogram', imgHist)
    
    # Calculate steering angle
    steering_angle = calculate_steering_angle(basePoint, w)
    print(f"Steering Angle: {steering_angle:.2f} degrees")
    
    # Display steering angle on the image
    cv2.putText(img, f'Steering Angle: {steering_angle:.2f} deg', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    return imgWarp

if __name__ == '__main__':
    # cap = cv2.VideoCapture('./video/track_vdo_1.mp4')
    cap = cv2.VideoCapture('./test_videos/vdo_41sec.mp4')

    initialTracbarVals = [162, 103, 33, 226]
    initialize_trackbars(initialTracbarVals)
    
    while cap.isOpened():
        success, img = cap.read()
        if not success:
            break
        
        img = cv2.resize(img, (480, 240))
        lane_image = get_lane_curve(img)
        cv2.imshow('Processed Video', lane_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

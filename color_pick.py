import cv2
import numpy as np

frameWidth = 640
frameHeight = 480

cap = cv2.VideoCapture('solidWhiteRight.mp4')

cap.set(3, frameWidth)
cap.set(4, frameHeight)

def empty(a): pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 240)

# Create trackbars for adjusting the HSV range
cv2.createTrackbar("HUE Min", "HSV", 0, 179, empty)
cv2.createTrackbar("HUE Max", "HSV", 179, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 0, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 0, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)

cap = cv2.VideoCapture('solidWhiteRight.mp4')  # Use video file or camera (index 1 for second camera)

frameCounter = 0

while True:
    frameCounter += 1

    # Check if we reached the end of the video and reset to the beginning
    if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        frameCounter = 0

    # Read the frame
    ret, img = cap.read()
    if not ret:
        break  # If no frame is read, exit the loop

    # Convert the frame to HSV
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Get trackbar positions for HSV values
    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    # Print the HSV values to track them
    print(f"Hue Min: {h_min}, Hue Max: {h_max}")
    print(f"Saturation Min: {s_min}, Saturation Max: {s_max}")
    print(f"Value Min: {v_min}, Value Max: {v_max}")

    # Create lower and upper bounds for the mask based on trackbar values
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Create a mask based on the current HSV range
    mask = cv2.inRange(imgHsv, lower, upper)

    # Apply the mask to the original image
    result = cv2.bitwise_and(img, img, mask=mask)

    # Convert the mask to BGR for visualization
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Stack the original image, mask, and result horizontally
    hStack = np.hstack([img, mask_bgr, result])

    # Display the stacked images
    cv2.imshow('Horizontal Stacking', hStack)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows() 
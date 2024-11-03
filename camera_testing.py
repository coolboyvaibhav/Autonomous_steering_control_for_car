import cv2

# Open a connection to the camera (0 is usually the default camera)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # If frame is read correctly, ret will be True
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break
    
    # Display the frame
    cv2.imshow('Live Feed', frame)
    
    # Press 'q' to exit the live feed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()\
cv2.destroyAllWindows()

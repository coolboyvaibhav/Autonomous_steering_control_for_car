# import cv2
# import numpy as np

# def region_of_interest(img, vertices):
#     mask = np.zeros_like(img)
#     cv2.fillPoly(mask, vertices, 255)
#     masked_image = cv2.bitwise_and(img, mask)
#     return masked_image

# def draw_lines(img, lines):
#     if lines is None:
#         return
#     img = np.copy(img)
#     line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    
#     for line in lines:
#         for x1, y1, x2, y2 in line:
#             cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 10)
    
#     img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
#     return img

# def process_frame(image):
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     blur = cv2.GaussianBlur(gray, (5, 5), 0)
#     edges = cv2.Canny(blur, 50, 150)

#     height, width = image.shape[:2]
#     region_vertices = [
#         (0, height),
#         (width // 2, height // 2),
#         (width, height)
#     ]
#     cropped_edges = region_of_interest(edges, np.array([region_vertices], np.int32))
    
#     lines = cv2.HoughLinesP(cropped_edges, rho=2, theta=np.pi/180, threshold=50, minLineLength=40, maxLineGap=150)
#     if lines is not None:
#         image_with_lines = draw_lines(image, lines)
#         return image_with_lines
#     else:
#         return image

# def main():
#     cap = cv2.VideoCapture(0)  # Replace 0 with the path to video if using pre-recorded video
    
#     while cap.isOpened():
#         ret, frame = cap.read()
#         if not ret:
#             break
        
#         frame = process_frame(frame)
#         cv2.imshow('Lane Detection', frame)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()
import cv2
import numpy as np
import math
import time
import RPi.GPIO as GPIO  # Assuming you're using a Raspberry Pi for servo control

# Servo Motor setup
servo_pin = 17  # GPIO pin connected to the servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency for servo
pwm.start(7.5)  # Initial position (centered)

# Function to control the servo motor
def set_servo_angle(angle):
    duty_cycle = 2.5 * angle / 18 + 2.5  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty_cycle)

# Function to define the region of interest (ROI)
def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    
    polygon = np.array([[
        (0, height),      
        (0, height // 2),  
        (width, height // 2), 
        (width, height)    
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

# Function to detect lane lines using Hough Transform
def detect_lines(image):
    lines = cv2.HoughLinesP(image, 1, np.pi / 180, 100, minLineLength=40, maxLineGap=5)
    
    if lines is None:
        return None
    
    left_lines = []
    right_lines = []
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else 0
            if slope < 0:  # Negative slope is a left line
                left_lines.append((x1, y1, x2, y2))
            else:  # Positive slope is a right line
                right_lines.append((x1, y1, x2, y2))
    
    return left_lines, right_lines

# Function to average the lines and return the lane positions
def average_lines(image, lines):
    left_lines, right_lines = lines
    left_line = np.mean(left_lines, axis=0) if left_lines else None
    right_line = np.mean(right_lines, axis=0) if right_lines else None

    if left_line is None or right_line is None:
        return None
    
    # Calculate the lane mid-point (center of the lanes)
    left_x1, left_y1, left_x2, left_y2 = left_line
    right_x1, right_y1, right_x2, right_y2 = right_line
    
    left_slope = (left_y2 - left_y1) / (left_x2 - left_x1)
    right_slope = (right_y2 - right_y1) / (right_x2 - right_x1)
    
    # Extend the lines to the bottom of the image
    height = image.shape[0]
    left_x1_new = int((height - left_y1) / left_slope + left_x1)
    right_x1_new = int((height - right_y1) / right_slope + right_x1)

    # Calculate the center of the road
    center_x = (left_x1_new + right_x1_new) // 2
    return center_x

# Main loop for processing video
cap = cv2.VideoCapture(0)  # Use webcam or video file

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Preprocess the image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Apply Region of Interest (ROI) to focus on the lower half
    roi_edges = region_of_interest(edges)

    # Detect lanes
    lines = detect_lines(roi_edges)
    
    if lines:
        center_x = average_lines(frame, lines)
        
        # Draw the lines on the frame for visualization
        left_lines, right_lines = lines
        for line in left_lines + right_lines:
            for x1, y1, x2, y2 in line:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw the center line (middle of the road)
        if center_x:
            cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (0, 0, 255), 2)
            
            # Calculate steering
            image_center = frame.shape[1] // 2
            offset = center_x - image_center
            
            # Map the offset to an angle for the servo
            if abs(offset) > 50:  # Only adjust if the car is far enough off-center
                angle = 90 + (offset * 0.1)  # 90 is the center angle for servo
                set_servo_angle(angle)
    
    # Show the frame with the detected lanes and steering info
    cv2.imshow('Lane Detection', frame)

    # Wait for a key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
pwm.stop()
GPIO.cleanup()

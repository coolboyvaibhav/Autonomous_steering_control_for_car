import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO for servo
GPIO.setmode(GPIO.BOARD)  # or GPIO.setmode(GPIO.BCM) based on pin numbering
servo_pin = 11  # Change to the pin you are using (e.g., 11 for BOARD or 17 for BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle (off)

# PID controller variables
Kp = 0.5  # Proportional gain
Ki = 0.001  # Integral gain
Kd = 0.01  # Derivative gain

previous_error = 0
integral = 0

# Function to set servo angle
def set_servo_angle(angle):
    # Map angle to duty cycle between 2.5 and 12.5%
    duty_cycle = 2.5 + (angle / 180.0) * 10.0
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow time for servo to reach position
    pwm.ChangeDutyCycle(0)  # Turn off the PWM signal (optional)

# Region of interest function (keeps focus on the relevant part of the image)
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

# Draw detected lines on the image
def draw_lines(img, lines):
    if lines is None:
        return
    img = np.copy(img)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 10)
    
    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    return img

# Function to calculate lane center
def find_lane_center(lines, img_width):
    left_line = []
    right_line = []
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            # Determine the lane sides based on the x-coordinates
            if x1 < img_width // 2 and x2 < img_width // 2:
                left_line.append((x1, y1, x2, y2))
            elif x1 > img_width // 2 and x2 > img_width // 2:
                right_line.append((x1, y1, x2, y2))
    
    # Calculate the center of the left and right lanes (using the average of x-coordinates)
    if left_line and right_line:
        left_x = np.mean([x1 for x1, _, x2, _ in left_line] + [x2 for _, _, x2, _ in left_line])
        right_x = np.mean([x1 for x1, _, x2, _ in right_line] + [x2 for _, _, x2, _ in right_line])
        
        lane_center = (left_x + right_x) / 2
        return lane_center
    return None

# Process the frame to detect lanes
def process_frame(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = image.shape[:2]
    region_vertices = [
        (0, height),
        (width // 2, height // 2),
        (width, height)
    ]
    cropped_edges = region_of_interest(edges, np.array([region_vertices], np.int32))
    
    lines = cv2.HoughLinesP(cropped_edges, rho=2, theta=np.pi/180, threshold=50, minLineLength=40, maxLineGap=150)
    
    lane_center = None
    if lines is not None:
        lane_center = find_lane_center(lines, width)
        image_with_lines = draw_lines(image, lines)
        if lane_center:
            # Draw the lane center line
            cv2.line(image_with_lines, (int(lane_center), 0), (int(lane_center), height), (0, 0, 255), 5)
        return image_with_lines, lane_center
    else:
        return image, lane_center

# PID controller function
def pid_control(error):
    global previous_error, integral
    
    # Proportional term
    P = Kp * error
    
    # Integral term
    integral += error
    I = Ki * integral
    
    # Derivative term
    D = Kd * (error - previous_error)
    
    # Total PID output
    pid_output = P + I + D
    
    # Update previous error
    previous_error = error
    
    return pid_output

# Main function to run the lane detection and control the servo
def main():
    cap = cv2.VideoCapture(0)  # Replace 0 with the path to video if using pre-recorded video
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame, lane_center = process_frame(frame)
        cv2.imshow('Lane Detection', frame)

        # Calculate the error (distance from the center of the lane to the center of the image)
        if lane_center:
            img_center = frame.shape[1] // 2
            error = lane_center - img_center

            # PID control to determine the steering angle
            pid_output = pid_control(error)

            # Control the servo based on the PID output
            servo_angle = 90 + int(np.clip(pid_output, -45, 45))  # Map PID output to servo angle [-45, 45]
            set_servo_angle(servo_angle)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    pwm.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    main()


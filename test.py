import cv2
import numpy as np
import math

video = cv2.VideoCapture(0) #'/dev/video0/'
video.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

def detect_edges(hsv):
    # White color filtering
    lower_white = np.array([0, 0, 200], dtype="uint8")
    upper_white = np.array([180, 25, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    edges = cv2.Canny(mask, 50, 100)
    cv2.imshow("edges", edges)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    cv2.imshow("roi", cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        print("No line segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit, right_fit = [], []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            
            slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
            if slope < 0 and x1 < left_region_boundary and x2 < left_region_boundary:
                left_fit.append((slope, intercept))
            elif slope > 0 and x1 > right_region_boundary and x2 > right_region_boundary:
                right_fit.append((slope, intercept))

    if left_fit:
        lane_lines.append(make_points(frame, np.average(left_fit, axis=0)))
    if right_fit:
        lane_lines.append(make_points(frame, np.average(right_fit, axis=0)))
    return lane_lines

def make_points(frame, line):
    height, _, _ = frame.shape
    slope, intercept = line
    y1 = height
    y2 = int(y1 / 2)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    return cv2.addWeighted(frame, 0.8, line_image, 1, 1)

def convert_to_HSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV", hsv)
    return hsv

while True:
    ret, frame = video.read()
    if not ret:
        print("Failed to capture video")
        break

    hsv = convert_to_HSV(frame)
    edges = detect_edges(hsv)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    
    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    cv2.imshow("Lane Lines", lane_lines_image)

    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
        break

video.release()
cv2.destroyAllWindows()


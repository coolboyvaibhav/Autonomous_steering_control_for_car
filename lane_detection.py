import cv2
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

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
    if lines is not None:
        image_with_lines = draw_lines(image, lines)
        return image_with_lines
    else:
        return image

def main():
    cap = cv2.VideoCapture(0)  # Replace 0 with the path to video if using pre-recorded video
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = process_frame(frame)
        cv2.imshow('Lane Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

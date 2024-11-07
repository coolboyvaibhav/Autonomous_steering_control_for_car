#code part 1 
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
#import numpy as np
#
## Read in the image
#image = mpimg.imread('test_images/camera.jpg')
#
## Grab the x and y size and make a copy of the image
#ysize = image.shape[0]
#xsize = image.shape[1]
#color_select = np.copy(image)
#
## Define color selection criteria
####### MODIFY THESE VARIABLES TO MAKE YOUR COLOR SELECTION
#red_threshold = 150
#green_threshold = 150
#blue_threshold = 150
#######
#
#rgb_threshold = [red_threshold, green_threshold, blue_threshold]
#
## Do a boolean or with the "|" character to identify
## pixels below the thresholds
#thresholds = (image[:,:,0] < rgb_threshold[0]) \
#            | (image[:,:,1] < rgb_threshold[1]) \
#            | (image[:,:,2] < rgb_threshold[2])
#color_select[thresholds] = [0,0,0]
##diplay image
#plt.imshow(image)
#plt.title("Input Image")
#plt.show()
#plt.imshow(color_select)
#plt.title("Color Selected Image")
#plt.show()
#
## Uncomment the following code if you are running the code locally and wish to save the image
## mpimg.imsave("test-after.jpg", color_select)

#working fine on the corridor but detecting multiple edges:--- part2
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
#import numpy as np
#import cv2  # OpenCV for corner detection
#
## Read in the image
#image = mpimg.imread('test_images/corridor.jpg')
#
## Grab the x and y size and make a copy of the image
#ysize = image.shape[0]
#xsize = image.shape[1]
#color_select = np.copy(image)
#
## Define color selection criteria (to select road color in the image)
#red_threshold = 100
#green_threshold = 100
#blue_threshold = 100
#rgb_threshold = [red_threshold, green_threshold, blue_threshold]
#
## Create a binary image where pixels below the threshold are set to black
#thresholds = (image[:,:,0] < rgb_threshold[0]) | (image[:,:,1] < rgb_threshold[1]) | (image[:,:,2] < rgb_threshold[2])
#color_select[thresholds] = [0, 0, 0]
#
## Convert the image to grayscale for corner detection
#gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
#
## Apply Canny edge detection to detect edges in the image
#edges = cv2.Canny(gray_image, 30, 150)
#
## Use Harris corner detection
## The Harris corner detection algorithm requires a grayscale image
## and a few other parameters for sensitivity
#corners = cv2.cornerHarris(edges, 2, 3, 0.04)
#
## Dilate the corners to make them more visible
#corners = cv2.dilate(corners, None)
#
## Threshold to mark the corners in a binary image (thresholding the corner response)
#image_with_corners = np.copy(image)
#image_with_corners[corners > 0.01 * corners.max()] = [255, 0, 0]  # Mark corners with red
#
## Display the images
#plt.figure(figsize=(12, 6))
#
## Original image
#plt.subplot(1, 3, 1)
#plt.imshow(image)
#plt.title("Original Image")
#
## Color selected image
#plt.subplot(1, 3, 2)
#plt.imshow(color_select)
#plt.title("Color Selected Image")
#import matplotlib.pyplot as plt

## Image with corners marked
#plt.subplot(1, 3, 3)
#plt.imshow(image_with_corners)
#plt.title("Corners Detected")
#
#plt.show()
#
## Optionally save the result
## mpimg.imsave("test-after-corners.jpg", image_with_corners)



#canny edge detection is tellig the perfectt edges in the corridor----part3
#import matplotlib.image as mpimg
#import matplotlib.pyplot as plt  # This imports the plotting library
#import numpy as np
#import cv2  # OpenCV for edge detection
#
## Read in the image
#image = mpimg.imread('test_images/corridor.jpg')
#
## Convert the image to grayscale
#gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
#
## Apply Gaussian Blur to reduce noise
#blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
#
## Apply Canny Edge Detection
#edges = cv2.Canny(blurred_image, 50, 150)
#
## Display the original and edge-detected images
#plt.figure(figsize=(10, 5))
#
## Show the original image
#plt.subplot(1, 2, 1)
#plt.imshow(image)
#plt.title("Original Image")
#
## Show the edge-detected image
#plt.subplot(1, 2, 2)
#plt.imshow(edges, cmap='gray')
#plt.title("Edge Detection (Canny)")
#
#plt.show()



# part 4 with reference generation 
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
#import numpy as np
#import cv2  # OpenCV for edge detection
#
## Read in the image
#image = mpimg.imread('test_images/corridor.jpg')
#
## Convert the image to grayscale
#gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
#
## Apply Gaussian Blur to reduce noise
#blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
#
## Apply Canny Edge Detection
#edges = cv2.Canny(blurred_image, 50, 150)
#
## Create a copy of the image to draw the reference line
#image_with_reference = np.copy(image)
#
## Get the center column of the image (horizontal center)
#height, width = edges.shape
#center_x = width // 2
#
## Initialize variables to hold the coordinates of the left and right edges
#left_edge_x = None
#right_edge_x = None
#
## Search for the first edge to the left and right of the center
## Start from the center and move left and right to find the first edge
#for offset in range(0, width // 2):  # Half the width of the image
#    # Look left
#    if center_x - offset >= 0 and edges[height // 2, center_x - offset] != 0:
#        left_edge_x = center_x - offset
#    # Look right
#    if center_x + offset < width and edges[height // 2, center_x + offset] != 0:
#        right_edge_x = center_x + offset
#
#    # If both left and right edges are found, exit the loop
#    if left_edge_x is not None and right_edge_x is not None:
#        break
#
## If edges are found, calculate the midpoint and draw the reference line
#if left_edge_x is not None and right_edge_x is not None:
#    # Calculate the midpoint between the left and right edges
#    middle_x = (left_edge_x + right_edge_x) // 2
#
#    # Draw a red line at the midpoint
#    cv2.line(image_with_reference, (middle_x, 0), (middle_x, height), (255, 0, 0), 2)
#
## Display the original and modified images
#plt.figure(figsize=(10, 5))
#
## Show the original image
#plt.subplot(1, 3, 1)
#plt.imshow(image)
#plt.title("Original Image")
#
## Show the image with the reference line
#plt.subplot(1, 3, 2)
#plt.imshow(image_with_reference)
#plt.title("Image with Reference Line")
## Show the image with CANNY EDGE
#plt.subplot(1, 3, 3)
#plt.imshow(edges, cmap='gray')
#plt.title("Edge detection")
#
#plt.show()

# ploting the edges that it is taking --part 5 
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread('test_images/Road.jpg') # Change the path to your image

# Check if image is loaded correctly
if image is None:
    print("Error loading image!")
    exit()

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian Blur to reduce noise
blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

# Apply Canny Edge Detection
edges = cv2.Canny(blurred_image, 50, 150)  

# Get the image dimensions
height, width = edges.shape

# Get the center column of the image (horizontal center)
center_x = width // 2

# Initialize variables to hold the coordinates of the left and right edges
left_edge_x = None
right_edge_x = None

# Search for the nearest left and right edges from the center
for offset in range(0, width // 2):  # Half the width of the image
    # Look left
    if center_x - offset >= 0 and edges[height // 2, center_x - offset] != 0:
        left_edge_x = center_x - offset
    # Look right
    if center_x + offset < width and edges[height // 2, center_x + offset] != 0:
        right_edge_x = center_x + offset

    # If both left and right edges are found, exit the loop
    if left_edge_x is not None and right_edge_x is not None:
        break

# If edges are found, calculate the midpoint and draw the reference line
if left_edge_x is not None and right_edge_x is not None:
    # Calculate the midpoint between the left and right edges
    middle_x = (left_edge_x + right_edge_x) // 2
# Draw the left and right lane lines in blue
    cv2.line(image, (left_edge_x, 0), (left_edge_x, height), (255, 0, 0), 2)
    cv2.line(image, (right_edge_x, 0), (right_edge_x, height), (255, 0, 0), 2)
    # Draw a green reference line at the midpoint across the image
    cv2.line(image, (middle_x, 0), (middle_x, height), (0, 255, 0), 2)
   

    # Draw blue edges on the original image
    for y in range(height):
        for x in range(left_edge_x, right_edge_x):
            if edges[y, x] != 0:
                image[y, x] = [255, 0, 0]  # Blue color in BGR

# Display the results
# Convert the image from BGR to RGB for displaying with matplotlib
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Plot the images using matplotlib
plt.figure(figsize=(10, 6))

# Show the original image with reference line and blue edges

plt.imshow(image_rgb)
plt.title("Image with Reference Line and Blue Edges")
plt.axis('off')

plt.show()

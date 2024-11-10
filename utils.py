import cv2
import numpy as np

def thresholding(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([80, 0, 0]) 
    uppperWhite = np.array([255, 160, 255])
    maskWhite = cv2.inRange(imgHsv, lowerWhite,uppperWhite)

    return  maskWhite

def warpImg(img,points,w,h,inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix=cv2.getPerspectiveTransform(pts2,pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

def nothing(a):
    pass

def initialize_trackbars(initialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, nothing)
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

def draw_points(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img


def getHistogram(img,minPer=0.1,display=False,region=1):
    if region==1:
        histValues=np.sum(img,axis=0)
    else:
        histValues=np.sum(img[img.shape[0]//region:,: ],axis=0)
    #histValues=np.sum(img,axis=0)
    #print(histValues)
    maxValue=np.max(histValues)
    #print(maxValue)
    minValue=minPer*maxValue

    indexArray=np.where(histValues>=minValue)
    basePoint=int(np.average(indexArray))
    #print(basePoint)
    if display:
        imgHist=np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x ,intensity in enumerate(histValues):
            cv2.line(imgHist,(x,img.shape[0]),(x,img.shape[0]-intensity//255//region),(255,0,255),1)
            cv2.circle(imgHist,(basePoint,img.shape[0]),20,(0,255,255),cv2.FILLED)
        return basePoint,imgHist
    
    return basePoint

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
 
# import numpy as np
# import cv2
# from utils import *
# import os
# import time
# import argparse

# parser = argparse.ArgumentParser()
# parser.add_argument('--model_cfg', type = str, default = '',
#                     help = 'Path to config file')
# parser.add_argument('--model_weights', type=str,
#                     default='',
#                     help='path to weights of model')
# parser.add_argument('--video', type=str, default='',
#                     help='path to video file')
# parser.add_argument('--src', type=int, default=0,
#                     help='source of the camera')
# parser.add_argument('--output_dir', type=str, default='',
#                     help='path to the output directory')
# args = parser.parse_args()

# # print the arguments
# print('----- info -----')
# print('[i] The config file: ', args.model_cfg)
# print('[i] The weights of model file: ', args.model_weights)
# print('[i] Path to video file: ', args.video)
# print('###########################################################\n')
# frameWidth= 640
# frameHeight = 480


# net = cv2.dnn.readNetFromDarknet('yolov3.cfg', 'yolov3.weights')

# # net = cv2.dnn.readNet(args.model_weights, args.model_cfg)
# classes = []
# with open("coco.names", "r") as f:
#     classes = [line.strip() for line in f.readlines()] # we put the names in to an array

# layers_names = net.getLayerNames()
# output_layers = [layers_names[i[0] -1] for i in net.getUnconnectedOutLayers()]
# colors = np.random.uniform(0, 255, size = (len(classes), 3))

# font = cv2.FONT_HERSHEY_PLAIN
# frame_id = 0
# cameraFeed= False
# #videoPath = 'road_car_view.mp4'
# cameraNo= 1
# #frameWidth= 640
# #frameHeight = 480


# if cameraFeed:intialTracbarVals = [24,55,12,100] #  #wT,hT,wB,hB
# else:intialTracbarVals = [42,63,14,87]   #wT,hT,wB,hB

# output_file = ''
# if cameraFeed:
#     cap = cv2.VideoCapture('./video/track_vdo_1.mp4')#cameraNo
#     cap.set(3, frameWidth)
#     cap.set(4, frameHeight)
# else:
#     cap = cv2.VideoCapture(args.video)
#     output_file = args.video[:-4].rsplit('/')[-1] + '_Detection.avi'
# count=0
# noOfArrayValues =10
# #global arrayCurve, arrayCounter
# arrayCounter=0
# arrayCurve = np.zeros([noOfArrayValues])
# myVals=[]
# initializeTrackbars(intialTracbarVals)


# #fourcc = cv2.VideoWriter_fourcc(*'XVID')
# #video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))
# video_writer = cv2.VideoWriter('output2.avi', cv2.VideoWriter_fourcc(*'XVID'), 
#     cap.get(cv2.CAP_PROP_FPS), (2 * frameWidth,frameHeight))
# starting_time = time.time()
# while True:

#     success, img = cap.read()
#     if not success:
#         print('[i] ==> Done processing!!!')
#         print('[i] ==> Output file is stored at', os.path.join(args.output_dir, output_file))
#         cv2.waitKey(1000)
#         break

#     #img = cv2.imread('test3.jpg')
#     if cameraFeed== False:img = cv2.resize(img, (frameWidth, frameHeight), None)
#     imgWarpPoints = img.copy()
#     imgFinal = img.copy()
#     imgCanny = img.copy()

#     imgUndis = undistort(img)
#     imgThres,imgCanny,imgColor = thresholding(imgUndis)
#     src = valTrackbars()
#     imgWarp = perspective_warp(imgThres, dst_size=(frameWidth, frameHeight), src=src)
#     imgWarpPoints = drawPoints(imgWarpPoints, src)
#     imgSliding, curves, lanes, ploty = sliding_window(imgWarp, draw_windows=True)

#     try:
#         curverad =get_curve(imgFinal, curves[0], curves[1])
#         lane_curve = np.mean([curverad[0], curverad[1]])
#         imgFinal = draw_lanes(img, curves[0], curves[1],frameWidth,frameHeight,src=src)

#         # Average
#         currentCurve = lane_curve // 50
#         if  int(np.sum(arrayCurve)) == 0:averageCurve = currentCurve
#         else:
#             averageCurve = np.sum(arrayCurve) // arrayCurve.shape[0]
#         if abs(averageCurve-currentCurve) >200: arrayCurve[arrayCounter] = averageCurve
#         else :arrayCurve[arrayCounter] = currentCurve
#         arrayCounter +=1
#         if arrayCounter >=noOfArrayValues : arrayCounter=0
#         cv2.putText(imgFinal, str(int(averageCurve)), (frameWidth//2-70, 70), cv2.FONT_HERSHEY_DUPLEX, 1.75, (0, 0, 255), 2, cv2.LINE_AA)

#     except:
#         lane_curve=00
#         pass

#     imgFinal= drawLines(imgFinal,lane_curve)

#     # Object detection 
#     success, frame = cap.read()

#     frame = cv2.resize(frame, (frameWidth, frameHeight), None)
#     frame_id += 1
#     height, width, channels = frame.shape
#     # Detect image
#     blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0,0,0), swapRB = True, crop = False)
#     net.setInput(blob)
#     start = time.time()
#     outs = net.forward(output_layers)

#     # Showing informations on the screen
#     class_ids = []
#     confidences = []
#     boxes = []
#     for out in outs:
#         for detection in out:
#             scores = detection[5:]
#             class_id = np.argmax(scores)
#             confidence = scores[class_id]
#             if confidence > 0.5:
#                 #Object detected
#                 center_x = int(detection[0] * width)
#                 center_y = int(detection[1] * height)
#                 w = int(detection[2] * width)
#                 h = int(detection[3] * height)

#                 # Rectangle coordinates
#                 x = int(center_x - w / 2)
#                 y = int(center_y -h / 2)
#                 #cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0))

#                 boxes.append([x, y, w, h])
#                 confidences.append(float(confidence))
#                 # Name of the object
#                 class_ids.append(class_id)

#     indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)

#     for i in range(len(boxes)):
#         if i in indexes:
#             x, y, w, h = boxes[i]
#             label = "{}: {:.2f}%".format(classes[class_ids[i]], confidences[i]*100)
#             color = colors[i]
#             cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
#             cv2.putText(frame, label, (x,y+10), font, 2, color, 2)

#     elapsed_time = time.time() - starting_time
#     fps = frame_id / elapsed_time
#     cv2.putText(frame, "FPS:" + str(fps), (10,30), font, 2, (0, 0, 0), 1)
#     imgBlank = np.zeros_like(img)
  
#     imgStacked = stackImages(0.7, ([imgUndis,frame],
#                                          [imgColor, imgCanny],
#                                          [imgWarp,imgSliding]
#                                          ))

#     #final_frame = cv2.hconcat((frame,imgCanny))
#     #video_writer.write(final_frame)
#     #cv2.imshow('frame',final_frame)
#     cv2.imshow("Image", frame)
#     cv2.imshow("PipeLine",imgStacked)
#     cv2.imshow("Result", imgFinal)


#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# #fourcc = cv2.VideoWriter_fourcc(*'MJPG')
# #out_corner = cv2.VideoWriter('img_corner_1.avi',fourcc, 20.0, (width, height))
# cap.release()
# cv2.destroyAllWindows()
# print('==> All done!')
# print('***********************************************************')


import cv2
import numpy as np
cap = cv2.VideoCapture('./video/track_vdo_1.mp4') 
# cap = cv2.VideoCapture(0)  # Open the webcam
cap.set(3, 640)  # Set frame width
cap.set(4, 480)  # Set frame height

while True:
    success, img = cap.read()  # Read the frame from the webcam
    
    if not success:
        print("Failed to grab frame")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detect edges using Canny edge detector
    edges = cv2.Canny(blur, 50, 150)
    
    # Create a mask for region of interest (ROI)
    mask = np.zeros_like(edges)
    height, width = mask.shape
    polygon = np.array([[
        (0, height),
        (width, height),
        (width // 2, height // 2)
    ]])
    cv2.fillPoly(mask, polygon, 255)
    
    # Mask the edges to only keep the region of interest
    masked_edges = cv2.bitwise_and(edges, mask)

    # Detect lines using Hough Transform
    lines = cv2.HoughLinesP(masked_edges, rho=6, theta=np.pi/60, threshold=160, lines=np.array([]), minLineLength=40, maxLineGap=25)

    # Create a blank image to draw the lines
    line_img = np.zeros_like(img)
    
    # Draw the lines on the line_img
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 0, 255), 10)

    # Combine the original image and the line image
    result = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    # Show the result
    cv2.imshow('Result', result)
    
    # Break the loop on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()  # Release the webcam
cv2.destroyAllWindows()  # Close all OpenCV windows

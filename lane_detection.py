import cv2
import numpy as np
import utils

curveList = []
avgVal = 10

def getLaneCurve(img, display=2):
    imgCopy = img.copy()
    imgResult = img.copy()
    
    # Step 1
    imgThres = utils.thresholding(img)

    # Step 2
    hT, wT, c = img.shape
    points = utils.val_trackbars()
    imgWarp = utils.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utils.draw_points(imgCopy, points)
        
    # Step 3
    middlePoint, imgHist = utils.getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = utils.getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    # Step 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    # Step 5 display
    if display != 0:
        imgInvWarp = utils.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 0))
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, (wT // 2 + (curve * 3), midY - 25), (wT // 2 + (curve * 3), midY + 25), (255, 0, 255), 5)
        
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10), 
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        
        # Uncomment below if you want FPS calculation
        # timer = cv2.getTickCount()
        # fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        # cv2.putText(imgResult, "FPS: " + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        if display == 2:
            imgStacked = utils.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                                 [imgHist, imgLaneColor, imgResult]))
            cv2.imshow('ImageStack', imgStacked)
        
        elif display == 1:
            cv2.imshow('Result', imgResult)

    return curve

if __name__ == '__main__':
    cap = cv2.VideoCapture('./Video/track_vdo_1.mp4')
    
    initialTrackBarVals = [184, 135, 48, 216]
    utils.initialize_trackbars(initialTrackBarVals)

    while True:
        frameCounter=1
        # Check if the video is opened successfully
        if not cap.isOpened():
            cap = cv2.VideoCapture('./Video/track_vdo_1.mp4')

        success, img = cap.read()
        
        # Reset to the start if the video ends
        if not success:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        img = cv2.resize(img, (480, 240))
        curve=getLaneCurve(img,display=0)
        print(curve)
        #cv2.imshow('Vid', img)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

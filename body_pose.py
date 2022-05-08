import time
import cv2 as cv
import numpy as np
import argparse
import serial 

#parser = argparse.ArgumentParser()
#parser.add_argument('--input', help='Path to image or video. Skip to capture frames from camera')
#parser.add_argument('--thr', default=0.2, type=float, help='Threshold value for pose parts heat map')
#parser.add_argument('--width', default=368, type=int, help='Resize input to specific width.')
#parser.add_argument('--height', default=368, type=int, help='Resize input to specific height.')

#args = parser.parse_args()
counter = 0
last_classified = ""
launchpad_port = serial.Serial(port='/dev/ttyACM1', baudrate=115200)

BODY_PARTS = {"Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
              "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
              "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
              "LEye": 15, "REar": 16, "LEar": 17, "Background": 18}

POSE_PAIRS = [["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
              ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
              ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
              ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
              ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]]

def detectStop(dict):
    #print(dict)
    if abs(getAngle(dict[1], dict[4])) < 20 and 180 - abs(getAngle(dict[1], dict[7])) < 20:
        print("Drive Far!")
        if not last_classified:
            global counter, last_classified
            last_classified = "stop"
        else:
            if last_classified == "stop":
                counter += 1
            else:
                last_classified = "stop"
                counter = 0
        #print(getAngle(dict[1], dict[4]), getAngle(dict[1], dict[7]))
        #print(dict)
    elif abs(getAngle(dict[1], dict[4])) < 20 and 180 - abs(getAngle(dict[1], dict[7])) > 20:
        print("Drive close")
        if not last_classified:
            global counter, last_classified
            last_classified = "right"
        else:
            if last_classified == "right":
                counter += 1
            else:
                last_classified = "right"
                counter = 0
    elif abs(getAngle(dict[1], dict[4])) > 20 and 180 - abs(getAngle(dict[1], dict[7])) < 20:
        print("LEFT!")
        if not last_classified:
            global counter, last_classified
            last_classified = "left"
        else:
            if last_classified == "left":
                counter += 1
            else:
                last_classified = "left"
                counter = 0
    #elif abs(dict[4][0] - dict[7][0]) < 1 and abs(dict[4][1] - dict[7][1]) < 1:
        #print(dict[4], dict[7])
        #print("Drive near")
    
    elif abs(dict[2][1] - dict[5][1]) == 0 and abs(dict[6][1] - dict[3][1]) == 0 and abs(dict[2][0] - dict[3][0]) == 0 and abs(dict[5][0] - dict[6][0]) == 0 :
        if not last_classified:
            global counter, last_classified
            last_classified = "folded arms"
        else:
            if last_classified == "folded arms":
                counter += 1
            else:
                last_classified = "folded arms"
                counter = 0
        print("Drive circle!")
    else:
        pass
        #print(getAngle(dict[1], dict[4]), getAngle(dict[1], dict[7]))
    

def getAngle(start, end):
    ang1 = np.arctan2(start[1] - end[1], start[0] - end[0]) * (180/np.pi)
    return ang1


threshold = 0.3

inWidth = 160
inHeight = 160

net = cv.dnn.readNetFromTensorflow("graph_opt.pb")

cap = cv.VideoCapture(0)
can_read = True
print("+++++++++++++START++++++++++++")
while cv.waitKey(1) < 0:
    if not can_read:
        can_read = True
    hasFrame, frame = cap.read()
    if not hasFrame:
        cv.waitKey()
        break

    frameWidth = frame.shape[1]
    frameHeight = frame.shape[0]

    net.setInput(cv.dnn.blobFromImage(frame, 1.0, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False))
    out = net.forward()
    out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements

    assert (len(BODY_PARTS) == out.shape[1])

    points = []
    pair_angle = [None for i in range(17)]
    for i in range(len(BODY_PARTS)):
        # Slice heatmap of corresponging body's part.
        heatMap = out[0, i, :, :]

        # Originally, we try to find all the local maximums. To simplify a sample
        # we just find a global one. However only a single pose at the same time
        # could be detected this way.
        _, conf, _, point = cv.minMaxLoc(heatMap)
        x = (frameWidth * point[0]) / out.shape[3]
        y = (frameHeight * point[1]) / out.shape[2]
        # Add a point if it's confidence is higher than threshold.
        points.append((int(x), int(y)) if conf > threshold else None)
    
    
    for pair_id, pair in enumerate(POSE_PAIRS):
        partFrom = pair[0]
        partTo = pair[1]
        assert (partFrom in BODY_PARTS)
        assert (partTo in BODY_PARTS)

        idFrom = BODY_PARTS[partFrom]
        idTo = BODY_PARTS[partTo]

        if points[idTo] and points[idFrom]:
            #print(points[idTo], points[idFrom])
            pair_angle[pair_id] = getAngle(points[idFrom], points[idTo])
            cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
            cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
    

    #t, _ = net.getPerfProfile()
    #freq = cv.getTickFrequency() / 1000
    #cv.putText(frame, '%.2fms' % (t / freq), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
    #print("Points", points)
    #print("Angle", pair_angle)
    if points[1] and points[4] and points[7] and points[5] and points[6] and points[2] and points[3]:
        detectStop(points)


    if counter == 5 and last_classified == "folded arms":
        msg = last_classified[0]
        launchpad_port.write(msg.encode('utf-8'))
        counter = 0
        print(msg)
        can_read = False

    if counter == 7:
        msg = last_classified[0]
        launchpad_port.write(msg.encode('utf-8'))
        counter = 0
        print(msg)
        can_read = False
        
    if launchpad_port.inWaiting():
            msg = launchpad_port.readline().decode('utf-8')
            print("Message: ", msg)
    #cv.imshow('OpenPose using OpenCV', frame)
    

print("==========END================")

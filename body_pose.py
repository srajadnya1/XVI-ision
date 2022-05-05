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
        print("STOP!")
        if not last_classified:
            global counter, last_classified
            last_classified = "stop"
        else:
            if last_classified == "stop":
                counter += 1
            else:
                last_classified = "stop"
                counter = 0
        print(getAngle(dict[1], dict[4]), getAngle(dict[1], dict[7]))
        #print(dict)
    elif abs(getAngle(dict[1], dict[4])) < 20 and 180 - abs(getAngle(dict[1], dict[7])) > 20:
        print("RIGHT!")
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
    else:
        pass
        #print(getAngle(dict[1], dict[4]), getAngle(dict[1], dict[7]))
    
"""
def getAngle(start, end):
    ang1 = np.arctan2(*start[::-1])
    ang2 = np.arctan2(*end[::-1])
    angle_deg = np.rad2deg((ang1 - ang2) % (2 * np.pi))
    if angle_deg > 180:
        return angle_deg % 360
    else:
        return angle_deg
"""

def getAngle(start, end):
    ang1 = np.arctan2(start[1] - end[1], start[0] - end[0]) * (180/np.pi)
    #angle_deg = np.rad2deg(ang1) % 360
    return ang1


threshold = 0.3

inWidth = 368
inHeight = 368

net = cv.dnn.readNetFromTensorflow("graph_opt.pb")

cap = cv.VideoCapture(1)
#time.sleep(10)
start_time = time.time()
end_time = start_time
print("+++++++++++++START++++++++++++")
while cv.waitKey(1) < 0:
#while end_time - start_time < 15:
    hasFrame, frame = cap.read()
    if not hasFrame:
        cv.waitKey()
        break

    frameWidth = frame.shape[1]
    frameHeight = frame.shape[0]

    #net.setInput(cv.dnn.blobFromImage(frame, 1.0, (inWidth, inHeight), (127.5, 127.5, 127.5), swapRB=False, crop=False))
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
            cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)

    t, _ = net.getPerfProfile()
    freq = cv.getTickFrequency() / 1000
    cv.putText(frame, '%.2fms' % (t / freq), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
    #print("Points", points)
    #print("Angle", pair_angle)
    if points[1] and points[4] and points[7]:
        detectStop(points)
    
    if counter >= 5:
        # msg = 'c'
        launchpad_port.write('c'.encode('utf-8'))

    end_time = time.time()
    cv.imshow('OpenPose using OpenCV', frame)

print("==========END================")
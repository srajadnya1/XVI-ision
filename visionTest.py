import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time

capture = cv2.VideoCapture(0)
face_detect = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
scale_percent = 40
launchpadPort = serial.Serial(port='COM5', baudrate=115200)

while True:

    ret, img = capture.read()
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)

    resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    faces = face_detect.detectMultiScale(gray, 1.0485258, 6)

    camera_center = (round(width/2), round(height/2))

    # cv2.circle(resized, (round(width/2), round(height/2)), 10, (255, 0, 0), 1)
    # cv2.imshow('live', resized)

    for (x, y, w, h) in faces:

        center_x = round(x + (w/2))
        center_y = round(y + (h/2))
        center_coord = (center_x, center_y)

        # print(center_coord, camera_center)

        if abs(camera_center[0] - center_x) < 20:
            launchpadPort.write('c'.encode('utf-8'))
            # launchpadPort.write((str(center_coord[0]) + 'c').encode('utf-8'))
            # print("centered")
        elif camera_center[0] - center_x > 0:
            launchpadPort.write('l'.encode('utf-8'))
            # launchpadPort.write((str(center_coord[0]) + 'l').encode('utf-8'))
            # print("left")
        elif camera_center[0] - center_x < 0:
            launchpadPort.write('r'.encode('utf-8'))
            # launchpadPort.write((str(center_coord[0]) + 'r').encode('utf-8'))
            # print("right")
        # else:
            # print("what the fuck")

        cv2.rectangle(resized, (x, y), (x+w, y+h), (255, 0, 0), 2)
        if launchpadPort.inWaiting():
            msg = launchpadPort.readline().decode('utf-8')
            print(msg)
        # cv2.circle(resized, center_coord, 10, (0, 255, 0), 1)
        cv2.imshow('facial_detection', resized)

    # time.sleep(1)

    if cv2.waitKey(1) & 0xff == ord('q'):
        launchpadPort.flush()
        break


capture.release()
cv2.destroyAllWindows()
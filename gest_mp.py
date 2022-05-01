import cv2
import mediapipe as mp
import time


def detect_pose(dict):
    down_list = [False, False, False, False, False]
    if dict[4][0] < dict[3][0] and dict[4][1] < dict[3][1]:
        #print("thumb is down")
        down_list[0] = True
    if dict[6][1] < dict[8][1]:
        #print("Index Down")
        down_list[1] = True
    if dict[11][1] < dict[12][1]:
        #print("Middle Down")
        down_list[2] = True
    if dict[15][1] < dict[16][1]:
        #print("Ring Down")
        down_list[3] = True
    if dict[19][1] < dict[20][1]:
        #print("Pinky Down")
        down_list[4] = True

    if not down_list.__contains__(True):
        cv2.putText(img,"Stop!", (10,100), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)
        print("Stop")
    elif down_list[0] == False and down_list[1] == True and down_list[2] == True and down_list[3] == True and down_list[4] == False:
        cv2.putText(img, "6!", (10, 100), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        print("6")
    elif down_list[0] == True and down_list[1] == False and down_list[2] == False and down_list[3] == True and down_list[4] == True:
        cv2.putText(img, "Peace!", (10, 100), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        print("Peace")
    elif down_list[0] == False and down_list[1] == False and down_list[2] == True and down_list[3] == True and down_list[4] == True:
        cv2.putText(img, "L!", (10, 100), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        print("L")


cap = cv2.VideoCapture(0)
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=1,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.2)
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0

while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        #print(results.multi_hand_landmarks[0])
        keypoint_dict = {}
        for handLms in results.multi_hand_landmarks:
            for id, lm in enumerate(handLms.landmark):
                #print(id,lm)
                h, w, c = img.shape
                cx, cy = int(lm.x *w), int(lm.y*h)
                keypoint_dict[id] = (cx, cy)
                #if id ==0:
                cv2.circle(img, (cx,cy), 3, (255,0,255), cv2.FILLED)

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
        #print(keypoint_dict)
        detect_pose(keypoint_dict)

    cTime = time.time()
    fps = 1/(cTime-pTime)
    pTime = cTime

    cv2.putText(img,str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)

    cv2.imshow("Image", img)
    cv2.waitKey(1)

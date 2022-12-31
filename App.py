# 
# Author : Keshav Chaudhary
# email  : k9971162104@gmail.com
# 
#imports START
import cv2
import mediapipe as mp
import time
import numpy as np
import math
#import END

cap = ''
try:
    # cap = cv2.VideoCapture('http://192.168.29.244:8080/video')
    cap = cv2.VideoCapture(0)
except:
    cap = cv2.VideoCapture(0)
    
mpHands = mp.solutions.hands            #just a formality
hands = mpHands.Hands()             #default values are good for right now
mpDraw = mp.solutions.drawing_utils

#for FPS
cTime = 0
pTime = 0

def distanceCalc(a,b):
    distance = (a[0]*a[0] - b[0]*b[0]) + (a[1]*a[1] - b[1]*b[1])
    if distance<0:
        distance=distance*(-1)
    if distance==0:
        return 0
    distance = math.sqrt(distance)
    return distance

def angleCalc(a,b,c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angleoutput = np.abs(radians*180.0/np.pi)
    if angleoutput > 180.0:
        angleoutput = 360.0 - angleoutput
    # print("Angle = "+str(angleoutput))
    return angleoutput

while True:
    success, img = cap.read()

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    # print(results.multi_hand_landmarks)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
            index_A = [handLms.landmark[8].x,handLms.landmark[8].y]
            index_B = [handLms.landmark[6].x,handLms.landmark[6].y]
            index_C = [handLms.landmark[5].x,handLms.landmark[5].y]
            if angleCalc(index_A, index_B, index_C) < 20:
            	print("Wrist Closed!")
#    print(results.multi_hand_landmarks.landmarks[0])
#    break
    cTime=time.time()
    fps = 1/(cTime-pTime)
    pTime=cTime
    cv2.putText(img, str(int(fps)),(10,70), cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),1)
    cv2.imshow("raw-img",img)

    cv2.waitKey(1)
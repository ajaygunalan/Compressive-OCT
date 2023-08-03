import cv2
import math
import numpy as np

cap = cv2.VideoCapture(2)

x_step = 0.4
y_step = 0.4
v = 0.00769*math.sqrt(x_step*x_step+y_step*y_step)/1000
print(v)
p = []

def onMouse(event, x, y, flag, param):
    global p
    if event == cv2.EVENT_LBUTTONDOWN:
        p.append([x,y])

cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', onMouse)

while True:
    _, frame = cap.read()
    frame = cv2.pyrDown(frame)
    if len(p)==2:
        dist = np.sqrt(pow((p[0][0]-p[1][0]),2) + pow((p[0][1]-p[1][1]),2))
        print(dist)
        cap.release()
        cv2.destroyAllWindows()
        break
    for each in p:
        frame=cv2.circle(frame,tuple([int(each[0]), int(each[1])]), 1, (0,0,255),2,cv2.LINE_AA)
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) == 27:
        cap.release()
        cv2.destroyAllWindows()
        break 

import cv2
import numpy as np
from matplotlib import pyplot as plt
import imutils

#load image
img = cv2.imread('/home/sli/calm_ws/laser_start.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#color definition
red_lower = np.array([0,0,255])
red_upper = np.array([255,255,255])

#red color mask (sort of thresholding, actually segmentation)
mask = cv2.inRange(hsv, red_lower, red_upper)

 # Finds the min & max element values and their positions.
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
maxLocList = []
maxLocList = list(maxLoc)
maxLocList[0] = maxLocList[0] + 10
maxLocList[1] = maxLocList[1] + 20
cv2.circle(img, maxLocList, 20, (0, 0, 255), 2, cv2.LINE_AA)
cv2.imshow('Track Laser', img)


#display image
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow('image', mask)
cv2.waitKey(0) & 0xFF
cv2.destroyAllWindows()

#print results
print ('number of dots, should be 4:',n)
print ('array of dot center coordinates:',array)
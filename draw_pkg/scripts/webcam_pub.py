#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
# from draw_pkg.msg import Calm_cor
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import itertools
import math
from scipy.interpolate import splprep, splev

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 1,
                       qualityLevel = 0.6,
                       minDistance = 11,
                       blockSize = 11 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 4,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
                  flags = (cv2.OPTFLOW_LK_GET_MIN_EIGENVALS))

# Start select points
start_select = False
draw_comp = False
# Draw line
drawing=False
drawcurve = False
user_points = np.empty([0,1,2], dtype=np.float32)
#size of laser pot
grid_size = 1
threshold=130

#find points inside lesion
def pointsinlesion(x,y,grid_size):
    y_max = y[0]
    for each in y:
        if each > y_max:
            y_max = each

    y_min = y[0]
    for each in y:
        if each < y_min:
            y_min = each

    x_max = x[0]
    for each in x:
        if each > x_max:
            x_max = each

    x_min = x[0]
    for each in x:
        if each < x_min:
            x_min = each

    y_l = y.tolist()
    x_l = x.tolist()
    x_l = [round(item, 2) for item in x_l]
    y_l = [round(item, 2) for item in y_l]
    points_inside = np.empty([0,2], dtype=np.float32)
    y_c = y_max - grid_size

    while y_c > y_min:
        y_c = round(y_c, 1)
        x_index_one = y_l.index(y_c)
        x_index_two = len(y_l) - 1 - y_l[::-1].index(y_c)
        p1 = [x_l[x_index_one]+grid_size, y_c]
        p2 = [x_l[x_index_two]-grid_size, y_c]
        num_line = abs(math.ceil((x_l[x_index_two] - x_l[x_index_one]) / grid_size)) - 1
        if num_line > 0:
            pi_line = np.linspace(start=p1, stop=p2, num=int(num_line))
        else:
            break
        points_inside = np.r_[points_inside, pi_line]
        y_c = y_c - grid_size
    print(points_inside)
    return points_inside

#select points for tracking
def sample_track_points(event, x, y, flags, param):
    global user_points, ix, iy, drawing, start_select, drawcurve
    
    if event == cv2.EVENT_LBUTTONDBLCLK:
        start_select = False
    
    elif event == cv2.EVENT_LBUTTONDOWN:
        drawing=True
        ix,iy=x,y
        
        
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing==True:
            user_points = np.empty([1,1,2], dtype=np.float32)
            user_points[0][0] = [x,y]
            
    elif event == cv2.EVENT_LBUTTONUP:
        drawing=False
        start_select = True
        drawcurve = True

def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cap = cv2.VideoCapture(2)
#    cap = cv2.VideoCapture('/home/sli/Downloads/IMG_0202.mov')
  _, frame = cap.read()
  first_level = cv2.pyrDown(frame)
  old_gray = cv2.cvtColor(first_level, cv2.COLOR_BGR2GRAY)
  p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params) 

  # set mouse call back
  cv2.namedWindow('Frame')
  cv2.setMouseCallback('Frame', sample_track_points)

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  while True:
      global user_points
      _, frame = cap.read()
      first_level = cv2.pyrDown(frame)
      gray_frame = cv2.cvtColor(first_level, cv2.COLOR_BGR2GRAY)


      p0 = np.concatenate([p0, user_points])
      p0 = np.float32(p0)

      user_points = np.empty([0,1,2])
              
      p1, st, error = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, p0, None, **lk_params)
          
      good_new = p1[st==1]

      # good_new = sorted(good_new, key=lambda k: [k[0]])
      good_new = np.array(good_new).reshape(-1, 1, 2)
      good_new = good_new.tolist()

      points_to_remove = []
      points_to_keep = []
      threshold=120
          

      combos = itertools.permutations(good_new, 2)
      for point1, point2 in combos:
          e = point1[0][0]
          f = point1[0][1]
          g = point2[0][0]
          h = point2[0][1]
          dis = math.dist([e, f], [g, h])
          if dis >= threshold:
              points_to_remove.append(point1)
                      
      for point in good_new:
          if point not in points_to_remove:
              points_to_keep.append(point)
                      

      good_new = points_to_keep
          
      good_new = np.array(good_new).reshape(-1, 1, 2)
      
      good_old = p0[st==1]
      
      for new in good_new:
          a, b = new.ravel()
          frame = cv2.circle(first_level, (int(a),int(b)), 3, (0, 255, 0), -1) 
      
      if drawcurve:
          xp = []
          yp = []
          for each in good_new:
              each = np.array(each).reshape(2,1)
              xp.append(each[0])
              yp.append(each[1])
          xp = np.array(xp)
          yp = np.array(yp)

          good_new = np.array(good_new).reshape(-1, 2)
          tck, u = splprep(good_new.T, u=None, s=0.0, per=1) 
          u_new = np.linspace(u.min(), u.max(), 50000)
          x_new, y_new = splev(u_new, tck, der=0)

          point_inside = pointsinlesion(x_new, y_new, grid_size)
          for new in point_inside:
              a, b = new.ravel()
              frame = cv2.circle(frame, (int(a),int(b)), 2, (225, 0, 0), -1) 


          i=0
          for each in x_new:
              x = each
              y = y_new[i]
              if i < (len(x_new)-1):
                  i=i+1
              frame = cv2.circle(frame, (int(x), int(y)), radius=1, color=(0, 0, 255), thickness=-1)
              
      old_gray = gray_frame.copy()
      p0 = good_new.reshape(-1, 1, 2)
      p0 = np.float32(p0)

      cv2.imshow('Frame', frame)
      
      key = cv2.waitKey(20)
      if key == 27:
          break

      if _:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(br.cv2_to_imgmsg(frame))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
  cv2.destroyAllWindows()
  cap.release()
 


if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass


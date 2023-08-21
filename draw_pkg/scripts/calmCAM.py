import cv2
  
vid = cv2.VideoCapture(2)
a  =  vid.getBackendName()
  
while(True):

    ret, frame = vid.read()

    cv2.imshow('CALM Video', frame)
      
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
vid.release()
cv2.destroyAllWindows()





# import cv2
# import numpy as np
# from scipy.interpolate import splprep, splev
# import itertools
# import math
# from scipy.ndimage import gaussian_filter1d
# import rospy
# import time
# from python_pkg import kalman
# from ralp_msgs.msg import teensy_input

# cap = cv2.VideoCapture(2)

# #ekf para
# A = 1  # No process innovation
# C = 1  # Measurement
# B = 0  # No control input
# Q = 0.5  # Process covariance
# R = 1  # Measurement covariance
# x = 100  # Initial estimate
# P = 1  # Initial covariance
# kalman_filter_x = kalman.SingleStateKalmanFilter(A, B, C, x, P, Q, R)
# kalman_filter_y = kalman.SingleStateKalmanFilter(A, B, C, x, P, Q, R)
# x_ekf = []
# y_ekf = []
# #Creat publisher-----------------------------------------------------
# pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
# rospy.init_node('ralp_msgs', anonymous=True)
# r = rospy.Rate(100) #100 hz
# msg = teensy_input()

# #global variables----------------------------------------------------
# # laser point
# laser_x=0
# laser_y=0
# # params for ShiTomasi corner detection
# feature_params = dict( maxCorners = 1,
#                        qualityLevel = 0.6,
#                        minDistance = 11,
#                        blockSize = 11 )

# # Parameters for lucas kanade optical flow
# lk_params = dict( winSize  = (15,15),
#                   maxLevel = 4,
#                   criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
#                   flags = (cv2.OPTFLOW_LK_GET_MIN_EIGENVALS))

# # Create old fame and find corners in it
# _, frame = cap.read()
# first_level = cv2.pyrDown(frame)
# # second_level = cv2.pyrDown(first_level)
# old_gray = cv2.cvtColor(first_level, cv2.COLOR_BGR2GRAY)
# # old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
# #p0 = [[[0, 0]]]
# # print(p0, p0.shape)
# # p0 = user_points = np.empty([0,1,2], dtype=np.float32)
# # mask = np.zeros_like(frame)

# # Start select points
# start_select = False
# draw_comp = False
# # Draw line
# drawing=False
# drawcurve = False
# user_points = np.empty([0,1,2], dtype=np.float32)
# #contour of lesion for track
# contour_drawing = False
# contour_points = np.empty([0,1,2], dtype=np.float32)
# #size of laser pot
# grid_size = 20
# threshold_select=220
# #points inside lesion for ablation
# sequance_point = []
# point_index = 0
# #msg for calm
# delta_x = 0
# delta_y = 0
# buttons = 0
# threshold_select_dist = 8

# #calibration angle
# scale = 197251
# theta_x = 27.099119243373767
# theta_x = math.radians(theta_x)
# theta_y = 65.27364226583315
# theta_y = math.radians(theta_y)
# #----------------------------------------------------------------------
# #record videos
# width_ori= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# height_ori= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# width_cont= int((width_ori+1)/2)
# height_cont= int((height_ori+1)/2)
# writer_ori= cv2.VideoWriter('test_ori.avi', cv2.VideoWriter_fourcc(*'FFV1'), 60, (width_ori,height_ori))
# writer_cont= cv2.VideoWriter('test_cont.avi', cv2.VideoWriter_fourcc(*'FFV1'), 60, (width_cont,height_cont))

# #save drawing contour
# draw_contour_metrics = []
# #save points inside lesion once flag
# pil_once = True
# #laser trajectory
# laser_points = []
# #calibration-----------------------------------------------------------
# def cali(calm, target):
#     x_step = 0.2
#     y_step = 0.2
#     v = math.sqrt(x_step*x_step+y_step*y_step)/1000

#     x_img=target[0]-calm[0]
#     y_img=target[1]-calm[1]

#     dx = (math.cos(theta_x)*x_img - math.cos(theta_y)*y_img) / scale
#     dy = (math.sin(theta_x)*x_img + math.sin(theta_y)*y_img) / scale
#     dy_step_pub = abs(1000*v/math.sqrt(1+(dx/dy)*(dx/dy))) * np.sign(dy)
#     dx_step_pub = abs((dx/dy)*dy_step_pub) * np.sign(dx)
#     dist_calm_tag = math.sqrt(x_img*x_img + y_img*y_img)
#     time = math.sqrt(dx*dx + dy*dy)/v

#     pub_coor = [dx_step_pub, dy_step_pub]
#     return pub_coor, time, dist_calm_tag
# # ----------------------------------------------------------------------

# # #find points inside lesion
# def pointsinlesion(x,y,grid_size):
#     y_max = y[0]
#     for each in y:
#         if each > y_max:
#             y_max = each

#     y_min = y[0]
#     for each in y:
#         if each < y_min:
#             y_min = each

#     x_max = x[0]
#     for each in x:
#         if each > x_max:
#             x_max = each

#     x_min = x[0]
#     for each in x:
#         if each < x_min:
#             x_min = each

#     y_l = y.tolist()
#     x_l = x.tolist()
#     x_l = [round(item, 1) for item in x_l]
#     y_l = [round(item, 1) for item in y_l]
#     points_inside = np.empty([0,2], dtype=np.float32)
#     y_c = y_max - grid_size - 30

#     while y_c > (y_min+30):
#         y_c = round(y_c, 1)
#         x_index_one = y_l.index(y_c)
#         x_index_two = len(y_l) - 1 - y_l[::-1].index(y_c)
#         p1 = [x_l[x_index_one]+grid_size + 30, y_c]
#         p2 = [x_l[x_index_two]-grid_size - 30, y_c]
#         num_line = abs(math.ceil((x_l[x_index_two] - x_l[x_index_one]) / 30)) - 1
#         if num_line > 1:
#             pi_line = np.linspace(start=p1, stop=p2, num=int(num_line))
#         else:
#             break
#         points_inside = np.r_[points_inside, pi_line]
#         y_c = y_c - grid_size
    
#     return points_inside
# #---------------------------------------------------------------------------------------------

# #select points for tracking
# def sample_track_points(event, x, y, flags, param):
#     global user_points, ix, iy, drawing, start_select, drawcurve, contour_points, contour_drawing
    
#     if event == cv2.EVENT_LBUTTONDBLCLK:
#         start_select = False
#         contour_points = np.empty([1,1,2], dtype=np.float32)
#         contour_points[0][0] = [x,y]
#         contour_drawing = True

#     elif event == cv2.EVENT_LBUTTONDOWN:
#         drawing=True
#         ix,iy=x,y
        
#     elif event == cv2.EVENT_MOUSEMOVE:
#         if drawing==True:
#             user_points = np.empty([1,1,2], dtype=np.float32)
#             user_points[0][0] = [x,y]
#             draw_contour_metrics.append((x,y))
            
#     elif event == cv2.EVENT_LBUTTONUP:
#         drawing=False
#         start_select = True
#         drawcurve = True
#         with open('/home/sli/calm_ws/drawing_points.txt', 'w') as file:
#             pointss = '\n'.join(str(e) for e in draw_contour_metrics)
#             file.write(pointss)
# #--------------------------------------------------------------------------------------------

# # set mouse call back
# cv2.namedWindow('Frame')
# cv2.setMouseCallback('Frame', sample_track_points)

# while not rospy.is_shutdown(): 
#     _, frame = cap.read()
#     writer_ori.write(frame)
# #laser point-------------------------------------------------------------------
#     frame_copy = frame.copy()
#     frame_copy = cv2.pyrDown(frame_copy)
#     frame_copy = cv2.GaussianBlur(frame_copy, (9,9), 0)
        
#     hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)

#         # lower mask (0-10)
#     lower_red = np.array([0,90,20])
#     upper_red = np.array([10,255,255])
#     mask0 = cv2.inRange(hsv, lower_red, upper_red)

#         # upper mask (170-180)
#     lower_red = np.array([160,95,20])
#     upper_red = np.array([180,255,255])
#     mask1 = cv2.inRange(hsv, lower_red, upper_red)

#         # join my masks
#     mask = mask0+mask1

#     kernel_7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
#     kernel_5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
#     kernel_3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
#         # erosion = cv2.erode(mask,kernel,iterations = 1)
#     opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_3, iterations=1)
#     dilation = cv2.dilate(opening,kernel_5,iterations = 1)
        
#     coord=cv2.findNonZero(dilation)
#     for each in coord:
#         laser_x = laser_x + each[0][0]
#         laser_y = laser_y + each[0][1]

#     if len(coord)>0:
#         laser_x = laser_x/(len(coord))
#         laser_y = laser_y/(len(coord))

#     kalman_filter_x.step(0, laser_x)
#     x_ekf = kalman_filter_x.current_state()
    
#     kalman_filter_y.step(0, laser_y)
#     y_ekf = kalman_filter_y.current_state()
    

#     # frame = cv2.circle(frame, tuple([int(laser_x),int(laser_y)]), 20, (0, 0, 255), 2, cv2.LINE_AA)
# #---------------------------------------------------------------------------------------
#     first_level = cv2.pyrDown(frame)
#     frame = cv2.circle(first_level, tuple([int(x_ekf), int(y_ekf)]), 3, (0, 0, 255), -1, cv2.LINE_AA)
#     # second_level = cv2.pyrDown(first_level)
#     gray_frame = cv2.cvtColor(first_level, cv2.COLOR_BGR2GRAY)
#     # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    

#     p0 = np.concatenate([p0, user_points])
#     p0 = np.float32(p0)

#     user_points = np.empty([0,1,2])
    
#     if contour_drawing:
#         p2, st1, error1 = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, contour_points, None, **lk_params)
    
#         contour_points_track = p2[st1==1]
#         for each in contour_points_track:
#             a, b = each.ravel()
#             frame = cv2.circle(first_level, (int(a),int(b)), 3, (0, 255, 0), -1)

#         contour_points = contour_points_track.reshape(-1, 1, 2)

#     p1, st, error = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, p0, None, **lk_params)
    

#     good_new = p1[st==1]

#     # good_new = sorted(good_new, key=lambda k: [k[0]])
#     good_new = np.array(good_new).reshape(-1, 1, 2)
#     good_new = good_new.tolist()
#     # good_new = delet_points(good_new)
#     points_to_remove = []
#     points_to_keep = []
    
        

#     combos = itertools.permutations(good_new, 2)
#     for point1, point2 in combos:
#         e = point1[0][0]
#         f = point1[0][1]
#         g = point2[0][0]
#         h = point2[0][1]
#         dis = math.dist([e, f], [g, h])
#         if dis >= threshold_select:
#             points_to_remove.append(point1)
                    
#     for point in good_new:
#         if point not in points_to_remove:
#             points_to_keep.append(point)
                    

#     good_new = points_to_keep
        
#     # good_new = sorted(good_new, key=lambda k: [k[0]])
#     good_new = np.array(good_new).reshape(-1, 1, 2)
#     good_old = p0[st==1]

#     for new in good_new:
#         a, b = new.ravel()
#         frame = cv2.circle(first_level, (int(a),int(b)), 3, (0, 255, 0), -1) 
    
#     if drawcurve:
#         xp = []
#         yp = []
#         for each in good_new:
#             each = np.array(each).reshape(2,1)
#             xp.append(each[0])
#             yp.append(each[1])
#         xp = np.array(xp)
#         yp = np.array(yp)

#         good_new = np.array(good_new).reshape(-1, 2)
#         tck, u = splprep(good_new.T, u=None, s=0.0, per=1) 
#         u_new = np.linspace(u.min(), u.max(), 50000)
#         x_new, y_new = splev(u_new, tck, der=0)

#         point_inside = pointsinlesion(x_new, y_new, grid_size)
#         #save once points inside lesion
#         if pil_once:
#             pil_metrics = point_inside
#             with open('/home/sli/calm_ws/points_inside.txt', 'w') as file:
#                 pointsss = '\n'.join(str(e) for e in pil_metrics)
#                 file.write(pointsss)
#             pil_once = False
#         for new in point_inside:
#             a, b = new.ravel()
#             frame = cv2.circle(frame, (int(a),int(b)), 3, (225, 0, 0), -1) 


#         i=0
#         for each in x_new:
#             x = each
#             y = y_new[i]
#             if i < (len(x_new)-1):
#                 i=i+1
#             frame = cv2.circle(frame, (int(x), int(y)), radius=1, color=(0, 0, 255), thickness=-1)
        
#         if point_index < (len(point_inside) - 1):
#             sequance_point = point_inside[point_index]
#             x_y_calm, time, dist_calmtag = cali(calm = (x_ekf, y_ekf), target = (sequance_point))
#             print((x_ekf, y_ekf), sequance_point, dist_calmtag)

#         if point_index < len(point_inside) and  dist_calmtag < threshold_select_dist:
#             laser_points.append((x_ekf,y_ekf))
#             point_index = point_index + 1

#         if point_index > (len(point_inside)-1):
#             with open('/home/sli/calm_ws/laser_tra.txt', 'w') as file:
#                 laser_p = '\n'.join(str(e) for e in laser_points)
#                 file.write(laser_p)
#             x_y_calm = (0,0)

#         msg.buttons = 0 
#         msg.deltax = x_y_calm[0]
#         msg.deltay = x_y_calm[1]
        
#         rospy.loginfo(msg)
#         pub.publish(msg)
#         # time.sleep(pause)


#     old_gray = gray_frame.copy()
#     p0 = good_new.reshape(-1, 1, 2)
#     p0 = np.float32(p0)
        
#     writer_cont.write(frame)
#     cv2.imshow('Frame', frame)
    
#     key = cv2.waitKey(20)
#     if key == 27:
#         break
    

    
# cv2.destroyAllWindows()
# cap.release()


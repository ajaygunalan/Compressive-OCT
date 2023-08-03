# import cv2
# import numpy as np

# # initialize the camera
# # If you have multiple camera connected with 
# # current device, assign a value in cam_port 
# # variable according to that
# cam_port = 2
# cam = cv2.VideoCapture(cam_port)
  
# # reading the input using the camera
# result, image = cam.read()
  
# # If image will detected without any error, 
# # show result
# if result:
  
#     # showing result, it take frame name and image 
#     # output
#     cv2.imshow("GeeksForGeeks", image)
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     lower_red = np.array([0, 0, 255])
#     upper_red = np.array([255, 255, 255])

#     # Checks if array elements lie between lower & upper red.
#     mask = cv2.inRange(hsv, lower_red, upper_red)
#     # saving image in local storage
#     cv2.imwrite("laser_start_mask.png", mask)
  
#     # If keyboard interrupt occurs, destroy image 
#     # window
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
#     cam.release()
    
# # If captured image is corrupted, moving to else part
# else:
#     print("No image detected. Please! try again")
    

  
import cv2
import numpy as np

path = '/home/sli/calm_ws/laser_start_y1.png'
image = cv2.imread(path, cv2.IMREAD_COLOR)
    # showing result, it take frame name and image 
    # output
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_red = np.array([0, 0, 255])
upper_red = np.array([255, 255, 255])

    # Checks if array elements lie between lower & upper red.
mask = cv2.inRange(hsv, lower_red, upper_red)
    # saving image in local storage
cv2.imwrite("laser_start_masky2.png", mask)
  
    # If keyboard interrupt occurs, destroy image 
    # window
cv2.waitKey(0)
cv2.destroyAllWindows()
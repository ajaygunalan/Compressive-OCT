import cv2
  
# initialize the camera
# If you have multiple camera connected with 
# current device, assign a value in cam_port 
# variable according to that
cam_port = 3
cam = cv2.VideoCapture(cam_port)
  
# reading the input using the camera
result, image = cam.read()
  
# If image will detected without any error, 
# show result
if result:
  
    # showing result, it take frame name and image 
    # output
    cv2.imshow("GeeksForGeeks", image)
  
    # saving image in local storage
    cv2.imwrite("/home/sli/regisAj1.png", image)
    #2 4 6 8 10 12 14 16
  
    # If keyboard interrupt occurs, destroy image 
    # window
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cam.release()
    
# If captured image is corrupted, moving to else part
else:
    print("No image detected. Please! try again")
    

  

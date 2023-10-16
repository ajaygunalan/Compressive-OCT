import cv2

cam_port = 2  # Replace this with the index of your camera if you have multiple cameras
cam = cv2.VideoCapture(cam_port)
frame_count = 0  # Initialize frame count for saving multiple frames

while True:
    result, frame = cam.read()
    
    if result:
        cv2.imshow("Live Feed", frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        # Break the loop if 'q' is pressed
        if key == ord('q'):
            break
        # Save the frame if 's' is pressed
        elif key == ord('s'):
            frame_name = f"frame_{frame_count}.png"
            cv2.imwrite(frame_name, frame)
            print(f"Saved {frame_name}")
            frame_count += 1
    else:
        print("No frame captured. Please try again.")
        
cv2.destroyAllWindows()
cam.release()

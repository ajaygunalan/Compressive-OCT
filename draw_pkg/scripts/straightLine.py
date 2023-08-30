#!/usr/bin/env python
import rospy
import numpy as np
import subprocess
from ralp_msgs.msg import teensy_input

def shutdown_hook():
    print("Inside Shutdown Hook - Interrupted by Ctrl+C, shutting down.")
    subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])

def sinusoidal_line(scan_time, max_velocity):
    rate_hz = 1.0 / scan_time
    rate = rospy.Rate(rate_hz)
    
    cord = np.arange(0, 2*np.pi, 0.1)
    v_range = max_velocity * np.sin(cord)
    
    msg = teensy_input()
    msg.buttons = 0
    msg.deltay = 0
    
    try:
        while not rospy.is_shutdown():
            for i in v_range:
                msg.deltax = i
                rospy.loginfo(msg)
                pub.publish(msg)
                rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    max_velocity = 20
    scan_time_ms = 10
    scan_time = scan_time_ms / 1000
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('sinusoidal_publisher', anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    sinusoidal_line(scan_time, max_velocity)




# numberOfPass = 2  # 1 Pass is equal to fornt and back to the same point
# pause = 2
# step = 0.8
# shortStep = 0.4
# shortPause = 0.1

# Arrow


# Single Line. Multiple Pass. Infinitle Pass
# try:
#     while not rospy.is_shutdown():
#         longLine(step, pause)
#         stop()
#         longLine(-step, pause)
#         stop()
# finally:
#     for i in range(0, 5):
#         stop()

# Finite Line
# for i in range(0, numberOfPass):
    # longLine(step, pause)
    # stop()
    # longLine(-step, pause)
    # stop()



# Rectangle
# for i in range(0, 12):
#     longLine(step, pause)
#     shortLine(shortStep, shortPause)
#     longLine(-step, pause)
#     shortLine(shortStep, shortPause)
# stop()

# Arrow
# longLine(step, pause)
# longLine(-step, pause)
# shortLine(step, pause)
# shortLine(-step, pause)
# diagonalLine(step, pause*2.5)

# 45 degree
# def diagonalLine(step, pause):
#     msg.buttons = 0 
#     msg.deltax = 0
#     msg.deltay = step
        
#     rospy.loginfo(msg)
#     pub.publish(msg)
#     time.sleep(pause)


# # 90 degree
# def shortLine(step, pause):
#     msg.buttons = 0 
#     msg.deltax = -step
#     msg.deltay = step
        
#     rospy.loginfo(msg)
#     pub.publish(msg)
#     time.sleep(pause)
    

# # 0 degree
# def longLine(step, pause):
#     msg.buttons = 0 
#     msg.deltax = step
#     msg.deltay = step
        
#     rospy.loginfo(msg)
#     pub.publish(msg)
#     time.sleep(pause)


# def stop():
#     msg.buttons = 1
#     msg.deltax = 0
#     msg.deltay = 0
        
#     rospy.loginfo(msg)
#     pub.publish(msg)
#     time.sleep(1)



# def circle():
#     max_velocity = 6
#     sleep = 0.1  #controls the size of the circle    
#     cord = np.arange(0, 2*np.pi, 0.1)
#     x_range = max_velocity *np.sin(cord)
#     y_range = max_velocity *np.cos(cord)

#     try:
#         while not rospy.is_shutdown():
#             for x, y in zip (x_range, y_range):  
#                 msg.buttons = 0 
#                 msg.deltax = x
#                 msg.deltay = y
        
#                 rospy.loginfo(msg)
#                 pub.publish(msg)
#                 time.sleep(sleep)
    
#     except rospy.ROSInterruptException:
#         pass
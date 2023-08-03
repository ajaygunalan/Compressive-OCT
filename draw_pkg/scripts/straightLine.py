#!/usr/bin/env python
import rospy
import time
import numpy as np
from ralp_msgs.msg import teensy_input
import math


# 45 degree
def diagonalLine(step, pause):
    msg.buttons = 0 
    msg.deltax = 0
    msg.deltay = step
        
    rospy.loginfo(msg)
    pub.publish(msg)
    time.sleep(pause)


# 90 degree
def shortLine(step, pause):
    msg.buttons = 0 
    msg.deltax = -step
    msg.deltay = step
        
    rospy.loginfo(msg)
    pub.publish(msg)
    time.sleep(pause)
    

# 0 degree
def longLine(step, pause):
    msg.buttons = 0 
    msg.deltax = step
    msg.deltay = step
        
    rospy.loginfo(msg)
    pub.publish(msg)
    time.sleep(pause)


def stop():
    msg.buttons = 0 
    msg.deltax = 0
    msg.deltay = 0
        
    rospy.loginfo(msg)
    pub.publish(msg)
    time.sleep(1)



def circle():
    max_velocity = 0.5
    sleep = 0.1  #controls the size of the circle    
    cord = np.arange(0, 2*np.pi, 0.1)
    x_range = max_velocity *np.sin(cord)
    y_range = max_velocity *np.cos(cord)

    try:
        while not rospy.is_shutdown():
            for x, y in zip (x_range, y_range):  
                msg.buttons = 0 
                msg.deltax = x
                msg.deltay = y
        
                rospy.loginfo(msg)
                pub.publish(msg)
                time.sleep(sleep)
    
    except rospy.ROSInterruptException:
        pass

    


pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
rospy.init_node('ralp_msgs', anonymous=True)
r = rospy.Rate(100) #100 hz
msg = teensy_input()


step = 0.4
pause = 2
shortStep = 0.4
shortPause = 0.1

# Arrow


# Single Line. Multiple Pass.
try:
    for i in range(0, 12000):
        if rospy.is_shutdown():
            break
        longLine(step, pause)
        stop()
        longLine(-step, pause)
        stop()
except rospy.ROSInterruptException:
    pass
finally:
    stop()


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
# circle()
# stop()

r.sleep()
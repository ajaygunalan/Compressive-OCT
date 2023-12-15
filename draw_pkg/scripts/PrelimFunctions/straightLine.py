#!/usr/bin/env python
import time
import rospy
import numpy as np
import subprocess
from ralp_msgs.msg import teensy_input


def shutdown_hook():
    print("Inside Shutdown Hook - Interrupted by Ctrl+C, shutting down.")
    subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])


# Clean cut at 2 watts, 100 freq, smart pulse
def rectangle():
    pause = 3
    step = 0.5
    shortStep = step/10
    shortPause = pause/8

    for i in range(0, 20):
        longLine(step, pause)
        shortLine(shortStep, shortPause)
        longLine(-step, pause)
        shortLine(shortStep, shortPause)
    stop()

if __name__ == '__main__':
    max_velocity = 10
    scan_time_ms = 10
    scan_time = scan_time_ms / 1000
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('sinusoidal_publisher', anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    # singleScan(scan_time, max_velocity)

    numberOfPass = 2  # 1 Pass is equal to fornt and back to the same point
    pause = 2
    step = 0.8
    shortStep = 0.4
    shortPause = 0.1

    msg = teensy_input()
    msg.buttons = 0
    msg.deltay = 0
    
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
        msg.buttons = 1
        msg.deltax = 0
        msg.deltay = 0
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(1)

    rectangle()
#!/usr/bin/env python
import time
import rospy
import numpy as np
import subprocess
import serial
from ralp_msgs.msg import teensy_input

def send_continuous_command(ser, command, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        ser.write(command)
        time.sleep(0.01) # You can adjust this to control how often the command is sent

def shutdown_hook():
    print("Inside Shutdown Hook - Interrupted by Ctrl+C, shutting down.")
    subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])

def rectangle():
    turn_laser_on()  
    pause = 1
    step = 1.0
    shortStep = step/10
    shortPause = pause/8
    for i in range(0, 40):
        longLine(step, pause)
        shortLine(shortStep, shortPause)
        longLine(-step, pause)
        shortLine(shortStep, shortPause)
    turn_laser_off()  
    stop()

if __name__ == '__main__':
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('laser_ablation', anonymous=True)
    rospy.on_shutdown(shutdown_hook)

    msg = teensy_input()
    
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

    rectangle()
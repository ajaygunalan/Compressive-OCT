#!/usr/bin/env python
import time
import rospy
import serial
import subprocess
import threading
from ralp_msgs.msg import teensy_input

# Initialize serial communication for laser control
port = "/dev/ttyACM0"  
baud_rate = 115200
laser_serial = serial.Serial(port, baud_rate)

# Global variable to control laser state
laser_on = False

def send_continuous_command(ser, command):
    global laser_on
    while laser_on:
        ser.write(command)
        time.sleep(0.01)

def start_laser():
    global laser_on
    laser_on = True
    threading.Thread(target=send_continuous_command, args=(laser_serial, bytes([1]))).start()

def stop_laser():
    global laser_on
    laser_on = False
    time.sleep(0.02)  # Allow some time for the thread to finish

def shutdown_hook():
    print("Inside Shutdown Hook - Interrupted by Ctrl+C, shutting down.")
    stop_laser()
    laser_serial.close()
    subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])

def rectangle():
    start_laser()
    duration = 1.0
    vel = 2.0
    shortVel = 0.05
    shortDuration = 0.5
    for i in range(0, 1):
        longLine(vel, duration)
        shortLine(vel, duration)
        # longLine(-vel, duration)
        # shortLine(shortVel, shortDuration)
    stop_laser()
    stop()

if __name__ == '__main__':
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('laser_ablation', anonymous=True)
    rospy.on_shutdown(shutdown_hook)

    msg = teensy_input()
    
    # 90 degree
    def shortLine(shortVel, shortDuration):
        msg.buttons = 0 
        msg.deltax = -shortVel
        msg.deltay = shortVel
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(shortDuration)
        
    # 0 degree
    def longLine(vel, duration):
        msg.buttons = 0 
        msg.deltax = vel
        msg.deltay = vel
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(duration)

    def stop():
        msg.buttons = 0
        msg.deltax = 0
        msg.deltay = 0
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(1)

    rectangle()
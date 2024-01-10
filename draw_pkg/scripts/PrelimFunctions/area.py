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
    time = 1.2
    vel = 1.0
    shortvel = vel/10
    shorttime = time/8
    for i in range(0, 40):
        longLine(vel, time)
        shortLine(shortvel, shorttime)
        longLine(-vel, time)
        shortLine(shortvel, shorttime)
    stop_laser()
    stop()

if __name__ == '__main__':
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('laser_ablation', anonymous=True)
    rospy.on_shutdown(shutdown_hook)

    msg = teensy_input()
    
    # 90 degree
    def shortLine(vel, time):
        msg.buttons = 0 
        msg.deltax = -vel
        msg.deltay = vel
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(time)
        
    # 0 degree
    def longLine(vel, time):
        msg.buttons = 0 
        msg.deltax = vel
        msg.deltay = vel
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(time)

    def stop():
        msg.buttons = 0
        msg.deltax = 0
        msg.deltay = 0
            
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(1)

    rectangle()
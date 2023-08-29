#!/usr/bin/env python
import rospy
import time
import numpy as np
import serial
import subprocess
from ralp_msgs.msg import teensy_input

# Laser control functions
def laser_on(ser, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        ser.write(bytes([1]))
        time.sleep(offTimer)

def laser_off(ser):
    ser.write(bytes([0]))

# Movement functions
def longLine(velocity, timeToTravel):
    msg.buttons = 0
    msg.deltax = velocity
    msg.deltay = velocity
    rospy.loginfo(msg)
    pub.publish(msg)
    time.sleep(timeToTravel)

def stop():
    msg.buttons = 0
    msg.deltax = 0
    msg.deltay = 0
    rospy.loginfo(msg)
    pub.publish(msg)
    time.sleep(stopTimer)

# Single scan function
def singleScan(velocity, timeToTravel):
    longLine(velocity, timeToTravel) # A to B
    longLine(-velocity, timeToTravel) # B to A


# Initialize all variables together
port = "/dev/ttyACM0"
baud_rate = 115200
ser = serial.Serial(port, baud_rate)
time_on = 3  # exposure time in seconds
scanTime = 30  # time for A to B and then to A in milliseconds
VelocityFactor = 0.01 # this can be adjusted based on actual measurements

stopTimer = 0.01
offTimer = 0.01

# Calculated variables
timeToTravel = scanTime/2000.0  # in milliseconds
scanLength = 4.6  # in mm
averageVelocity = scanLength / timeToTravel  # in mm/s
CALMVelocity = averageVelocity * VelocityFactor  # in mm/s
ScanNo = int(time_on * 1000 / scanTime)  # Number of scans

# Print them for user confirmation
print("Exposure time:", time_on, "s")
print("Scan time from A to B and to A:", scanTime, "ms")

print("Scan length:", scanLength, "mm")
print("Velocity Factor:", VelocityFactor)
print("Number of scans:", ScanNo)
print("Average velocity:", averageVelocity, "mm/s")

print("Calculated CALM velocity:", CALMVelocity, "mm/s")
print("Calculated Time to travel from A to B:", timeToTravel, "s")
CALMVelocity = 14 # 
print("Actual CALM velocity:", CALMVelocity)
timeToTravel  = 0.1 # s
print("Actual Time to travel from A to B:", timeToTravel, "s")

user_input = input("Is everything okay? (yes/no): ")

if user_input.lower() == "yes":
    # ROS setup
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('ralp_msgs', anonymous=True)
    r = rospy.Rate(100)
    msg = teensy_input()

    try:
        laser_on(ser, time_on)
        while not rospy.is_shutdown():
            for i in range(ScanNo):
                if rospy.is_shutdown():
                    break
                singleScan(CALMVelocity, timeToTravel)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")
    except rospy.ROSInterruptException:
        pass
    finally:
        stop()
        laser_off(ser)
        ser.close()
        subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])
else:
    print("Exiting...")

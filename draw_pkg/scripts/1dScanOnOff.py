#!/usr/bin/env python
import rospy
import time
import numpy as np
import serial
import subprocess
import threading
import sched
from ralp_msgs.msg import teensy_input

# Initialize variables
port, baud_rate = "/dev/ttyACM0", 115200
ser = serial.Serial(port, baud_rate)
time_on = 3
scan_time_ms = 1000
scan_time = scan_time_ms / 1000
rate = 1000000  # Update velocity rate (Hz)
VelocityFactor, offTimer = 0.01, 0.01
scanLength = 4.6
averageVelocity = scanLength / scan_time
CALMVelocity = averageVelocity * VelocityFactor
ScanNo = int(time_on/scan_time)

# Initialize ROS & scheduler
rospy.init_node('ralp_msgs', anonymous=True)
s = sched.scheduler(time.time, time.sleep)
pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
msg = teensy_input()

# Utility functions
# Print Configuration
def print_config():
    print("Exposure time:", time_on, "s")
    print("Scan time from A to B and to A:", scan_time_ms, "ms")
    print("Scan length:", scanLength, "mm")
    print("Velocity Factor:", VelocityFactor)
    print("Number of scans:", ScanNo)
    print("Average velocity:", averageVelocity, "mm/s")
    print("Calculated CALM velocity:", CALMVelocity, "mm/s")
    print("Actual CALM velocity:", CALMVelocity)

def execute_at_time(func, args, exec_time):
    s.enterabs(exec_time, 1, func, args)
    s.run()

def laser_on(ser, duration, offTimer):
    end_time = time.time() + duration
    while time.time() < end_time:
        execute_at_time(ser.write, (bytes([1]),), offTimer)

def laser_off(ser):
    execute_at_time(ser.write, (bytes([0]),), 0)

def do_scan(velocity):
    msg.buttons, msg.deltax, msg.deltay = 0, velocity, 0
    rospy.loginfo(msg)
    pub.publish(msg)

def singleScan(scan_time, max_velocity):
    s = sched.scheduler(time.time, time.sleep)
    steps = int(scan_time * rate)
    cord = np.linspace(0, 2*np.pi, steps)
    v_range = max_velocity * np.sin(cord)
    time_interval = 1.0 / rate
    start_time = time.time()
    for i, v in enumerate(v_range):
        execute_at_time(do_scan, (v,), start_time + (i * time_interval))

def main_thread():
    try:
        laser_on(ser, time_on, offTimer)
        for _ in range(ScanNo):
            if rospy.is_shutdown(): break
            singleScan(scan_time, CALMVelocity)
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        laser_off(ser)
        ser.close()
        subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])

# User confirmation and main execution
print_config()
if input("Is everything okay? (yes/no): ").lower() == "yes":
    threading.Thread(target=main_thread).start()
else:
    print("Exiting...")
#!/usr/bin/env python
import rospy
import time
import numpy as np
import serial
import subprocess
import threading
import sched
from ralp_msgs.msg import teensy_input

# Initialize ROS node in the main thread
rospy.init_node('ralp_msgs', anonymous=True)

# Initialize scheduler
s = sched.scheduler(time.time, time.sleep)

def laser_on(ser, duration, offTimer):
    end_time = time.time() + duration
    def turn_laser_on():
        ser.write(bytes([1]))
    while time.time() < end_time:
        s.enter(offTimer, 1, turn_laser_on, ())
        s.run()

def laser_off(ser):
    s.enter(0, 1, ser.write, (bytes([0]),))
    s.run()

def do_scan(velocity):
    msg.buttons = 0
    msg.deltax = velocity
    msg.deltay = 0
    rospy.loginfo(msg)
    pub.publish(msg)

def singleScan(scan_time, max_velocity):
    global s
    s = sched.scheduler(time.time, time.sleep)  # Reset scheduler
    steps = int(scan_time * rate)
    cord = np.linspace(0, 2*np.pi, steps)
    v_range = max_velocity * np.sin(cord)
    print("velocity range: ", v_range)

    time_interval = 1.0 / rate
    start_time = time.time()

    for i, v in enumerate(v_range):
        print("Time: ", start_time + (i * time_interval))
        print("Veloc: ", v)
        s.enterabs(start_time + (i * time_interval), 1, do_scan, (v,))
        
    s.run()

def main_thread():
    global pub, msg
    try:
        laser_on(ser, time_on, offTimer)
        for i in range(ScanNo):
            if rospy.is_shutdown():
                break
            singleScan(scan_time, CALMVelocity)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")
    except rospy.ROSInterruptException:
        pass
    finally:
        laser_off(ser)
        ser.close()
        subprocess.run(["rosrun", "draw_pkg", "calmStop.py"])

# Initialize all variables
port = "/dev/ttyACM0"
baud_rate = 115200
ser = serial.Serial(port, baud_rate)
time_on = 3
scan_time_ms = 100
scan_time = scan_time_ms / 1000
scan_fz = 1.0 / scan_time
rate = 1000  # Hz the rate at which we update velocity per second
VelocityFactor = 0.01
offTimer = 0.01

# Calculated variables
scanLength = 4.6
averageVelocity = scanLength / scan_time
CALMVelocity = averageVelocity * VelocityFactor
ScanNo = int(time_on/ scan_time)
ScanNo = 100

# Print for user confirmation
print("Exposure time:", time_on, "s")
print("Scan time from A to B and to A:", scan_time_ms, "ms")
print("Scan length:", scanLength, "mm")
print("Velocity Factor:", VelocityFactor)
print("Number of scans:", ScanNo)
print("Average velocity:", averageVelocity, "mm/s")
print("Calculated CALM velocity:", CALMVelocity, "mm/s")
CALMVelocity = 20
print("Actual CALM velocity:", CALMVelocity)

# ROS setup
pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
msg = teensy_input()

# User confirmation
user_input = input("Is everything okay? (yes/no): ")
if user_input.lower() == "yes":
    main_thread_instance = threading.Thread(target=main_thread)
    main_thread_instance.start()
else:
    print("Exiting...")
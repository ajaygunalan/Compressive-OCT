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

def singleScan(timeToTravel, max_velocity):
    rate = 10  # 10Hz, i.e., 0.1s per cycle
    steps = int(timeToTravel * rate)
    cord = np.linspace(0, 2*np.pi, steps)
    v_range = max_velocity * np.sin(cord)

    def do_scan(velocity, direction):
        msg.buttons = 0
        msg.deltax = direction * velocity
        msg.deltay = 0
        rospy.loginfo(msg)
        pub.publish(msg)

    for i, v in enumerate(v_range):
        s.enter(i * 0.1, 1, do_scan, (v, 1))
    
    for i, v in enumerate(reversed(v_range)):
        s.enter((steps + i) * 0.1, 1, do_scan, (v, -1))

    s.run()


def main_thread():
    global pub, msg
    try:
        laser_on(ser, time_on, offTimer)
        while not rospy.is_shutdown():
            for i in range(ScanNo):
                if rospy.is_shutdown():
                    break
                singleScan(timeToTravel, CALMVelocity)
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
scanTime = 30
VelocityFactor = 0.01
offTimer = 0.01


# Calculated variables
timeToTravel = scanTime / 2000.0
scanLength = 4.6
averageVelocity = scanLength / timeToTravel
CALMVelocity = averageVelocity * VelocityFactor
ScanNo = int(time_on * 1000 / scanTime)
stopTimer = timeToTravel*1

# Print for user confirmation
print("Exposure time:", time_on, "s")
print("Scan time from A to B and to A:", scanTime, "ms")
print("Scan length:", scanLength, "mm")
print("Velocity Factor:", VelocityFactor)
print("Number of scans:", ScanNo)
print("Average velocity:", averageVelocity, "mm/s")
print("Calculated CALM velocity:", CALMVelocity, "mm/s")
print("Calculated Time to travel from A to B:", timeToTravel, "s")
print("Calculated Time break time:", stopTimer, "s")
CALMVelocity = 6
print("Actual CALM velocity:", CALMVelocity)
timeToTravel  = 0.1
print("Actual Time to travel from A to B:", timeToTravel, "s")

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

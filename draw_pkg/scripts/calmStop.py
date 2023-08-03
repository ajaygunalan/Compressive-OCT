#!/usr/bin/env python
import rospy
import time
from ralp_msgs.msg import teensy_input

def talker():
    pub = rospy.Publisher('ralp_msgs/teensy_input', teensy_input, queue_size=10)
    rospy.init_node('ralp_msgs', anonymous=True)
    r = rospy.Rate(100) #100 hz
    
    msg = teensy_input()

    step = 0.00
    pause = 1
    flag = 1 
    while not rospy.is_shutdown():
        msg.buttons = flag
        msg.deltax = step
        msg.deltay = step
        
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(pause)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
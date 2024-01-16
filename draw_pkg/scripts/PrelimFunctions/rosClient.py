import rospy
from oct_msgs.srv import Depth

# Initialize a ROS node
rospy.init_node('depth_client')

# Wait for the service to be available
rospy.wait_for_service('/estimate_depth')

# Create a handle to call the service
estimate_depth = rospy.ServiceProxy('/estimate_depth', Depth)

try:
    while not rospy.is_shutdown():
        try:
            # Call the service and print the response
            resp = estimate_depth()
            print("Depth received: ", resp.depth)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # Add a delay if you want to limit the frequency of service calls
        rospy.sleep(1)
except KeyboardInterrupt:
    print("Interrupted by user, shutting down.")
finally:
    # Any cleanup code goes here
    pass
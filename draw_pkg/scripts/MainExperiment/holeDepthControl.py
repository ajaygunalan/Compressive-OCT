import serial
import time
import rospy
from oct_msgs.srv import Depth

# Parameters
port = "/dev/ttyACM1"
baud_rate = 115200
desired_depth = 10.0
Kp = 0.5
Kd = 0.2
prev_error = 0.0
max_time_on = 2.0  # Hard maximum limit for time_on
initial_ablation_time = 0.5  # in seconds

# Function to send continuous command
def send_continuous_command(ser, command, duration):
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < duration:
        ser.write(command)
        time.sleep(0.01)

# Initialize ROS node
rospy.init_node('depth_client')
rospy.wait_for_service('/estimate_depth')
estimate_depth = rospy.ServiceProxy('/estimate_depth', Depth)

try:
    with serial.Serial(port, baud_rate) as ser:
        print(ser.name)
        print("Press Ctrl+C to stop execution and turn off the laser.")

        # Perform initial ablation without checking depth
        print("Performing ablation...")
        send_continuous_command(ser, bytes([1]), initial_ablation_time)  # initial ablation time
            
        # Turn off laser
        print("Laser OFF")
        ser.write(bytes([0]))
        
        while not rospy.is_shutdown():

            # Check depth
            resp = estimate_depth()
            current_depth = resp.depth
            print("Current depth: ", current_depth)

            if current_depth < desired_depth:
                error = desired_depth - current_depth
                print("Depth difference: ", error)
                derivative = error - prev_error
                time_on = min(Kp * error + Kd * derivative, max_time_on)
                print("Calculated time_on: ", time_on)

                proceed = input('Type "ok" to proceed: ')
                if proceed.lower() == "ok":
                    print("Laser ON")
                    send_continuous_command(ser, bytes([1]), time_on)

                print("Laser OFF")
                ser.write(bytes([0]))
            
            else:
                print("Desired depth reached. Exiting.")
                break

            prev_error = error
            rospy.sleep(1)

except KeyboardInterrupt:
    print("\nTurning off laser and stopping execution...")
    ser.write(bytes([0]))
    print("Execution stopped.")

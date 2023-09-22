import serial
import time
import rospy
from oct_msgs.srv import Depth

# Parameters
port = "/dev/ttyACM0"
baud_rate = 115200
desired_depth = 2.0
Kp = 0.5
Kd = 0.2
prev_error = 0.0
max_time_on = 2.0  # Hard maximum limit for time_on
initial_ablation_time = 1.0 # in seconds

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

print("CALM Client established connection with OCT Server.")
print(f"Initial ablation time: {initial_ablation_time} seconds")
print(f"Desired Depth: {desired_depth} mm")
proceed_experiment = input('Do you want to proceed with the experiment? (yes/no): ')

if proceed_experiment.lower() != 'yes':
    print("Exiting as user did not grant permission for the experiment.")
    exit()

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
            print("Requested depth from OCT Server")
            resp = estimate_depth()
            current_depth = resp.depth
            print("Current depth: ", current_depth)

            if current_depth.data < desired_depth:
                error = desired_depth - current_depth.data
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

import serial
import time
import rospy
import csv
from oct_msgs.srv import Depth

# Function to save data to CSV
def save_to_csv(row):
    with open('ablation_data.csv', 'a') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(row)

# Parameters
port = "/dev/ttyACM0"
baud_rate = 115200
Kp = 0.5
Kd = 0.2
prev_error = 0.0
max_time_on = 2.0
initial_ablation_time = float(input("Enter initial ablation time (in seconds): "))
desired_depth = float(input("Enter desired depth (in mm): "))
laser_power = float(input("Enter laser power (in W): "))
laser_frequency = float(input("Enter laser frequency (in Hz): "))

# Initialize CSV
with open('ablation_data.csv', 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Timestamp', 'Initial Ablation Time', 'Desired Depth', 'Initial Depth', 'Final Depth', 'Error', 'Calculated Time ON', 'Kp', 'Kd', 'Laser Power', 'Laser Frequency'])

# Initialize ROS
rospy.init_node('depth_client')
rospy.wait_for_service('/estimate_depth')
estimate_depth = rospy.ServiceProxy('/estimate_depth', Depth)

proceed_experiment = input('Do you want to proceed with the experiment? (yes/no): ')

if proceed_experiment.lower() != 'yes':
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
        current_depth = resp.depth.data
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


            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            save_to_csv([timestamp, None, desired_depth, initial_depth, final_depth, error, time_on, Kp, Kd, laser_power, laser_frequency])

            print("Laser OFF")
            ser.write(bytes([0]))
            
        else:
            print("Desired depth reached. Exiting.")
            break

        prev_error = error
        rospy.sleep(1)

except KeyboardInterrupt:
    ser.write(bytes([0]))
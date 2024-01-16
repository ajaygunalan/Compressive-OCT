import serial
import time
import rospy
import csv
from oct_msgs.srv import Depth
from datetime import datetime

# Function to send continuous command
def send_continuous_command(ser, command, duration):
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < duration:
        ser.write(command)
        time.sleep(0.01)

# Function to save data to CSV
def save_to_csv(filename, row):
    with open(filename, 'a') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(row)

# Parameters
port = "/dev/ttyACM1"
baud_rate = 115200
Kp = 0.5
Kd = 0.5
tolerance = 0.05 # mm
prev_error = 0.0
max_time_on = 2.0
desired_depth = 1.5

initial_ablation_time = float(input("Enter initial ablation time (in seconds): "))
experiment_trial = input("Enter the experiment trial number: ")
filename = f'/home/sli/OCTAssistedSurgicalLaserWS/src/data/log_{experiment_trial}.csv'
proceed_experiment = input('Do you want to proceed with the experiment? (yes/no): ')

if proceed_experiment.lower() != 'yes':
    print("Exiting as user did not grant permission for the experiment.")
    exit()


# Initialize CSV
with open(filename, 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Desired Depth', 'Initial Ablation Time', 'Kp', 'Kd', 'Tolerance'])
    csvwriter.writerow([desired_depth, initial_ablation_time, Kp, Kd, tolerance])
    csvwriter.writerow([])
    csvwriter.writerow(['Timestamp', 'Current Depth', 'Error', 'Derivative Error', 'Calculated Time ON'])

# Initialize ROS
rospy.init_node('depth_client')
rospy.wait_for_service('/estimate_depth')
estimate_depth = rospy.ServiceProxy('/estimate_depth', Depth)

try:
    with serial.Serial(port, baud_rate) as ser:
        print(ser.name)
        print("Press Ctrl+C to stop execution and turn off the laser.")
        
        # Perform initial ablation without checking depth
        print("Performing Initial ablation...")
        send_continuous_command(ser, bytes([1]), initial_ablation_time)
        ser.write(bytes([0]))

        # Turn off laser
        print("Laser OFF")
        ser.write(bytes([0]))

    while not rospy.is_shutdown():
        # Check depth
        print("Requested depth from OCT Server")
        resp = estimate_depth()
        current_depth = resp.depth.data
        error = desired_depth - current_depth
        derivative  = error - prev_error

        print("Desired depth: ", desired_depth)
        print("Current depth: ", current_depth)
        print("Depth Error difference: ", error)
            
        if current_depth< desired_depth -tolerance:
            ser = serial.Serial(port, baud_rate)

            time_on = min(Kp * error + Kd * derivative, max_time_on)
            print("Calculated time_on: ", time_on)
            proceed = input('Type "ok" to proceed: ')

            if proceed.lower() == "ok":
                print("Laser ON")
                send_continuous_command(ser, bytes([1]), time_on)

            print("Laser OFF")
            ser.write(bytes([0]))
            ser.close()

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            save_to_csv(filename, [timestamp, current_depth, error, derivative, time_on])
        
        else:
            print("Desired depth reached. Exiting.")
            if not (time_on):
                time_on = initial_ablation_time
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            save_to_csv(filename, [timestamp, current_depth, error, derivative, time_on])

            break
        
        prev_error = error
        rospy.sleep(1)

except KeyboardInterrupt:
    if 'ser' in locals():
        ser.write(bytes([0]))
        ser.close()
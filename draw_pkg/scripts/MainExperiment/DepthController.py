import serial
import time
import rospy
import csv
from oct_msgs.srv import Depth
from datetime import datetime

# Function to send continuous command
def send_continuous_command(ser, command, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        ser.write(command)
        time.sleep(0.01) # You can adjust this to control how often the command is sent

# Function to save data to CSV
def save_to_csv(filename, row):
    with open(filename, 'a') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(row)

# Parameters
port = "/dev/ttyACM1"
baud_rate = 115200
Kp = 2.0
Kd = 2.0
prev_error = 0.0
desired_depth = 0.5
initial_ablation_time = 0.6  # sec why it doesnt work with 0.5 or less
mini_ablation_time = 0.2  # sec
round_off_precision = 4
default_time_off = 2.0


# initial_ablation_time = float(input("Enter initial ablation time (in seconds): "))
experiment_trial = input("Enter the experiment trial number: ")
filename = f'/home/sli/OCTAssistedSurgicalLaserWS/src/data/log_{experiment_trial}.csv'
proceed_experiment = input('Do you want to proceed with the experiment? (ok): ')

if proceed_experiment.lower() != 'ok':
    exit()
    

# Initialize CSV
with open(filename, 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Desired Depth', 'Initial Ablation Time', 'Kp', 'Kd'])
    csvwriter.writerow([desired_depth, initial_ablation_time, Kp, Kd])
    csvwriter.writerow([])
    csvwriter.writerow(['Timestamp', 'Current Depth', 'Error', 'Derivative Error', 'Calculated Time ON'])

# Initialize ROS
rospy.init_node('depth_client')
rospy.wait_for_service('/estimate_depth')
estimate_depth = rospy.ServiceProxy('/estimate_depth', Depth)

try:
    with serial.Serial(port, baud_rate) as ser:
        print(ser.name)
        print("Serial Port is activated")
        print("Press Ctrl+C to stop execution and turn off the laser.")
        print("Performing Initial ablation...")
        send_continuous_command(ser, bytes([1]), initial_ablation_time)
        print("Laser OFF")
    
    while not rospy.is_shutdown():
        # Check depth
        print("Requested depth from OCT Server")
        resp = estimate_depth()
        current_depth = round(resp.depth.data, round_off_precision)  
        error = round(desired_depth - current_depth, round_off_precision)
        derivative  = round(error - prev_error, round_off_precision)
        print(f"Desired depth: {desired_depth:.4f}")
        print(f"Current depth: {current_depth:.4f}")
        print(f"Depth Error difference: {error:.4f}")
            
        if current_depth < desired_depth:
            ser = serial.Serial(port, baud_rate)
            time_on = round(max((Kp * error) + (Kd * derivative), mini_ablation_time), round_off_precision)  
            print(f"Calculated time_on: {time_on:.4f}")
            proceed = input('Type "ok" to proceed: ')
            if proceed.lower() == "ok":
                print("Laser ON")
                send_continuous_command(ser, bytes([1]), time_on)

            print("Laser OFF")
            send_continuous_command(ser, bytes([0]), default_time_off)

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            save_to_csv(filename, [timestamp, current_depth, error, derivative, time_on])
        
        else:
            print("Desired depth reached. Exiting.")
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            time_on = -143
            save_to_csv(filename, [timestamp, current_depth, error, derivative, time_on])
            break
        
        prev_error = error
        rospy.sleep(1)

except KeyboardInterrupt:
    if 'ser' in locals():
        ser.write(bytes([0]))
        ser.close()

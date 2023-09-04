# Vary the laser exposure time from 100, 200, 300, 400, 500, 600, 700, 800, 900 milliseconds, 
# and 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 seconds. 
# Measure the corresponding depth using OCT images. 
# For each data point, conduct the experiment 10 times to obtain 
# the mean, median, and standard deviation for the ablated depth.

import serial
import time

# Change the COM port to match your configuration
port = "/dev/ttyACM0"
baud_rate = 115200
time_on = 1  # Duration for laser ON


def send_continuous_command(ser, command, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        ser.write(command)
        time.sleep(0.01) # You can adjust this to control how often the command is sent

try:
    with serial.Serial(port, baud_rate) as ser:
        print(ser.name)  # check which port was really used
        print("Press Ctrl+C to stop execution and turn off the laser.")
        while True:
            print("Laser ON")
            send_continuous_command(ser, bytes([1]), time_on) # Turn laser on for time_on seconds

            print("Laser OFF")
            send_continuous_command(ser, bytes([0]), time_off) # Turn laser off for time_off seconds

except KeyboardInterrupt:
    # This block is executed when Ctrl+C is pressed
    print("\nTurning off laser and stopping execution...")
    with serial.Serial(port, baud_rate) as ser:
        ser.write(bytes([0])) # Turn laser off by sending '0'
    print("Execution stopped.")
import serial
import time

# Change the COM port to match your configuration
port = "/dev/ttyACM0"
baud_rate = 115200
time_on = 3  # Duration for laser ON
time_off = 3 # Duration for laser OFF

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

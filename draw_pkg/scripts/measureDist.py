import subprocess
import time

def main(delay_time):
    # Record start time
    start_time = time.perf_counter()

    # First command
    cmd1 = "rostopic pub /ralp_msgs/teensy_input ralp_msgs/teensy_input '{buttons: 1, deltax: -60.0, deltay: 0.0}'"
    subprocess.Popen(cmd1, shell=True)

    # Calculate elapsed time and remaining delay
    elapsed_time = time.perf_counter() - start_time
    remaining_delay = delay_time - elapsed_time

    # Wait for the remaining time to complete the delay
    time.sleep(max(0, remaining_delay))

    # Second command
    cmd2 = "rostopic pub /ralp_msgs/teensy_input ralp_msgs/teensy_input '{buttons: 1, deltax: -40.0, deltay: 0.0}'"
    subprocess.Popen(cmd2, shell=True)

if __name__ == "__main__":
    delay_time = 1.0  # You can change this value
    main(delay_time)


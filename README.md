## Compressive OCT for Fast Depth Measurement in Laser Microsurgery

This document describes the use of 3D Optical Coherence Tomography (OCT) for acquiring and segmenting surface details in laser microsurgery. 
The process utilizes ThorLabs' SpectralRadar SDK 5.2 for data capture, which is then exported to MATLAB as CSV files.
A Compressive Depth Map is reconstructed in MATLAB using TVAL3 and compared with a fully scanned depth map for accuracy.

### Usage Instructions

#### MATLAB
- Run `sparseOCTdepth.m`
- The user should input the experimental trial number (`trialNum`) when prompted.
- Note: **Avoid** using `0` as the trial number, as this is the default directory reserved for C++ operations
- Data will be stored in `./data/getDepthFromSparse3Doct/<trialNum>`.

#### C++ (Visual Studio)
- Running the C++ program in Visual Studio will store data in the directory `./data/getDepthFromSparse3Doct/0`

#### C++ Setup
1. [Setting Up C++ on Windows](https://www.youtube.com/watch?v=1OsGXuNA5cc)
2. [Optimal Visual Studio Setup for C++ Projects](https://www.youtube.com/watch?v=qeH9Xv_90KM)
3. [Static Linking of ThorLabs API](https://www.youtube.com/watch?v=or1dAmUO8k0) (Static linking is preferred for speed).
   - ThorLabs API can be found [here](https://gitlab.advr.iit.it/BRL/laser/thorlabs-api).
4. Ensure to build for x64 architecture.
     

Will be updated in the future!

### To Do
1. Abalted contour detection:
  - https://www.cvat.ai/annotation-service
  - https://www.makesense.ai/
  - https://segment-anything.com/

### Known Bugs

1. The first plot in `matlabCustomFunction\calibration.m` is skewed. Running it again resolves this issue.
2. `OCTImageCapture.exe` doesn't generate `scanPatternX.jpg` properly. The X varies each time.


### Auto-CALM

**For a successful operation, first turn on CALM and then connect it via USB. Verify that it moves smoothly with the pen; if not, it may exhibit vibration.**

### Part 1: Setting Up CALM with ROS
1. Start the ROS core with the command `roscore`.
2. Ensure that the black-wire USB cable from the CALM control box is connected to your laptop.
3. Check the available USB devices by running [this script](https://gist.github.com/ajaygunalan/0c7afbe4a931fb4fb3f9de0dd223f763#file-findusbdev-sh), and you should see `/dev/ttyACM0 - Teensyduino_USB_Serial_6311670`.
4. Navigate to `home/sli/calm/catkinWs` and execute `source devel/setup.bash`.
5. Integrate CALM with the ROS network using the command:
   ```
   rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
   ```
   You should see the following output:
   ```
   (base) sli@iitadvrlw009u:~/calm/catkinWs$ rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
   [INFO] [1702387249.376261]: ROS Serial Python Node
   [INFO] [1702387249.380185]: Connecting to /dev/ttyACM0 at 115200 baud
   [INFO] [1702387251.486989]: Requesting topics...
   [INFO] [1702387251.739470]: Note: subscribe buffer size is 512 bytes
   [INFO] [1702387251.740479]: Setup subscriber on /ralp_msgs/teensy_input [ralp_msgs/teensy_input]
   ```

### Part 2: Verifying CALM Motion
1. Navigate to `/home/sli/OCTAssistedSurgicalLaserWS` and execute `source devel/setup.bash`.
2. To move CALM, use the command:
   ```
   rostopic pub /ralp_msgs/teensy_input ralp_msgs/teensy_input "{buttons: 1, deltax: 0.1, deltay: 0.1}"
   ```
3. To stop CALM, run `rosrun draw_pkg calmStop.py`.

### Part 3: Verifying CALM Camera
1. Ensure that the video USB is connected and recognized as `/dev/video2 - TerraTec_Electronic_GmbH_TerraTec_Grabby`.
2. Navigate to `/home/sli/OCTAssistedSurgicalLaserWS/src/draw_pkg/scripts/PrelimFunctions` and execute `python3 liveCALMcamera.py`.
3. Ensure that you deactivate any active `conda` environment, and feel free to adjust the white balance (WB) and other camera settings for optimal image quality.

### Part 4: CALM Laser Activation
1. Confirm that the CALM on/off USB is connected and recognized as `/dev/ttyACM1 - Arduino__www.arduino.cc__0043_55731323636351500152`.
2. Run `python3 laseronoff.py` from `/home/sli/OCTAssistedSurgicalLaserWS/src/draw_pkg/scripts/PrelimFunctions`.
3. Activate the CO2 laser by pressing the foot pedal.
4. If you need guidance on using the Deka Laser, consult Leo. Smart Pulse mode is suitable for clean cuts, while Continuous Laser mode may result in less precise cuts.

### Part 5: Auto CALM Operation
1. Navigate to `/home/sli/calm/catkinWs` and execute `source devel/setup.bash`.
2. Run the script `rosrun python_pkg final.py`.
3. Draw a circle starting from the bottom.
4. Move in a clockwise direction.
5. Ensure that the feature is clear, and the background is clean.
6. Turn on the light source.
7. Monitor the density of the red spot.


### OCT Lenses
1. [LSM03](https://www.thorlabs.com/thorproduct.cfm?partnumber=LSM03)
2. [LSM04](https://www.thorlabs.com/thorproduct.cfm?partnumber=LSM04)

<img src="OCTLens.JPG"  width="800" height="400">

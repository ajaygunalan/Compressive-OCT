
Will be updated in the future!

### To Do
1. Abalted contour detection:
  - https://www.cvat.ai/annotation-service
  - https://www.makesense.ai/
  - https://segment-anything.com/

### Known Bugs

1. The first plot in `matlabCustomFunction\calibration.m` is skewed. Running it again resolves this issue.
2. `OCTImageCapture.exe` doesn't generate `scanPatternX.jpg` properly. The X varies each time.


### Auto-CALM Notes
1. Go to `/home/sli/calm/catkinWs`  and `source /devel/setup.bash`
2. run `rosrun python_pkg final.py`
3. Draw the circle starting from bottom.
4. Go in clockwise driection
5. Make sure feature is clear and backgound is neat.
6. open light
7. density of red spot
6. first turn on calm then connect with usb and check it is moving with pen. if not, it will **vibrate**.


### OCT Lenses
1. [LSM03](https://www.thorlabs.com/thorproduct.cfm?partnumber=LSM03)
2. [LSM04](https://www.thorlabs.com/thorproduct.cfm?partnumber=LSM04)

<img src="OCTLens.JPG"  width="800" height="400">

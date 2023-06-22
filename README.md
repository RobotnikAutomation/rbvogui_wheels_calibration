# robotnik_wheels_calibration

Node created to calibrate the wheels and write the mean values in the corresponding .env file.

To calibrate the wheels once, run:

roslaunch robotnik_wheels_calibration wheel_calibration.launch calibrate_once:=true

The node can be kept alive setting the argument calibrate_once to false. Then, to calibrate the wheels, a service 'calibrate_wheels' should be called.

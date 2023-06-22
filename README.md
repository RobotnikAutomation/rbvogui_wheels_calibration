# rbvogui_wheels_calibration

Node created to calibrate the wheels and write the mean values in the corresponding .env file.

To calibrate the wheels once, run:
```
roslaunch rbvogui_wheels_calibration wheel_calibration.launch calibrate_once:=true
```

The node can be kept alive setting the argument calibrate_once to false. Then, to calibrate the wheels, a service 'calibrate_wheels' should be called:

```
rosservice call /calibrate_wheels "{}"
```

The node is 30 seconds saving the data, then computes the means and saves those means in the corresponding .env file.

To apply the changes, the software should be restarted.

## Parameters
- **file_name**: Name of the .env file (default: robot_params.env)
- **file_path**: Path to the file (default: /home/robot/robot_params)
- **env_var**: Name of the environment variable to modify (default: ROBOT_JOINT_POTENTIOMETER_VOLTAGE_CALIBRATION)
- **topic_sub**: Topic needed to get the analog_inputs from the wheels (default: robotnik_base_hw/io)
- **calibration_duration**: Time the node is saving data beforte computing the mean (default: 30)
- **calibrate_once**: Boolean variable used to calibrate only once and then let the node die (default: false)

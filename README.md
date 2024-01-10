# rbvogui_wheels_calibration

Node created to calibrate the wheels and write the mean values in the corresponding .env file.

## Installation
```
cd && cd catkin_ws/src
```
```
git clone https://github.com/RobotnikAutomation/rbvogui_wheels_calibration.git
```
```
catkin build rbvogui_wheels_calibration
```
```
source ../devel/setup.bash
```

## Calibration
To calibrate the wheels once, run:
```
roslaunch rbvogui_wheels_calibration wheel_calibration.launch calibrate_once:=true
```

The node can be kept alive setting the argument calibrate_once to false. Then, to calibrate the wheels, a service 'calibrate_wheels' should be called:

```
rosservice call /calibrate_wheels "{}"
```

The node is 60 seconds saving the data, then computes the means and saves those means in the corresponding .env file.

To apply the changes, the software should be restarted.

## Parameters
- **file_name**: Name of the .env file (default: summit_xl_params.env)
- **file_path**: Path to the file (default: /home/robot/robot_params/base)
- **env_var**: Name of the environment variable to modify (default: ROBOT_JOINT_POTENTIOMETER_VOLTAGE_CALIBRATION)
- **topic_sub**: Topic needed to get the analog_inputs from the wheels (default: robot/robotnik_base_hw/io)
- **calibration_duration**: Time the node is saving data beforte computing the mean (default: 60)
- **calibrate_once**: Boolean variable used to calibrate only once and then let the node die (default: false)
- **save_to_file**: Boolean variable used to save the values in the corresponding .env file (default: false)
- **use_median**: Boolean variable used to choose the median instead of the mean value (default: false)
- **avoid_repeated_data**: Boolean variable used to avoid consecutive repeated data (default: true)
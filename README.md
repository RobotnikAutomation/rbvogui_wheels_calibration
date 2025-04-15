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

The node is 30 seconds saving the data, then computes the means and saves those means in the corresponding .env file.

To apply the changes, the software should be restarted.

## Parameters
- **file_name**: Name of the .env file (default: robot_params.env)
- **file_path**: Path to the file (default: /home/robot/robot_params)
- **env_var**: Name of the environment variable to modify (default: ROBOT_JOINT_POTENTIOMETER_VOLTAGE_CALIBRATION)
- **topic_sub**: Topic needed to get the analog_inputs from the wheels (default: robot/robotnik_base_hw/io)
- **calibration_duration**: Time the node is saving data before computing the mean (default: 60)
- **calibrate_once**: Boolean variable used to calibrate only once and then let the node die (default: false)
- **save_to_file**: Boolean variable used to select if the obtained values are written in the env file (default: false)
- **use_median**: Boolean variable used to select median values for calibration (default: true). If false, it uses the mean values.
- **restart_base_hw**: Boolean variable used to select if the hardware controller has to be restarted after calibration (default: true)
- **bringup_nodes_to_restart**: List with the names of the nodes to be restarted if *restart_base_hw* is set (default: [robotnik_base_hw, controller_spawner])
- **rosmon_node**: Rosmon node whose service is called to restart *bringup_nodes_to_restart* (default: rosmon_bringup)
- **robot_namespace**: Robot namespace (default: robot)
- **ros_param**: Name of the ros parameter to be set before restarting *bringup_nodes_to_restart* (default: /robot/robotnik_base_hw/joint_potentiometer_voltage_calibration)
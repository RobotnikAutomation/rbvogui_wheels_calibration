# rbvogui_wheels_calibration

ROS 2 package (Jazzy) that calibrates the steering wheels of an RBVogui robot by recording analog sensor data over a configurable period and writing the resulting values into the corresponding `.env` file.

## Installation
```bash
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/RobotnikAutomation/rbvogui_wheels_calibration.git
cd ~/robot_ws
colcon build --packages-select rbvogui_wheels_calibration
source install/setup.bash
```

## Calibration

### One-shot calibration
To calibrate the wheels once and let the node exit automatically:
```bash
ros2 launch rbvogui_wheels_calibration wheel_calibration.launch.py calibrate_once:=true
```

### Persistent mode
Keep the node alive and trigger calibration on demand via service call:
```bash
ros2 launch rbvogui_wheels_calibration wheel_calibration.launch.py
```
```bash
ros2 service call /calibrate_wheels std_srvs/srv/Trigger
```

The node records data for `calibration_duration` seconds, then computes the median (or mean) of the analog inputs and writes the result into the `.env` file under the `ROBOT_STEERING_CALIBRATION` variable.

To apply the changes, restart the robot software.

### Monitoring status
```bash
ros2 topic echo /wheel_calibration/status
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_model` | `$ROBOT_MODEL` env var (`rbvogui`) | Robot model, used to derive the default file name |
| `file_name` | `<robot_model>_params.env` | Name of the `.env` file to modify |
| `file_path` | `/home/robot/robot_settings/robot_params/base` | Directory containing the `.env` file |
| `env_var` | `ROBOT_STEERING_CALIBRATION` | Environment variable to update inside the file |
| `topic_sub` | `/robot/robotnik_io_controller/io` | Topic providing `robotnik_io_msgs/InputsOutputs` data |
| `calibration_duration` | `60` | Seconds to collect data before computing the result |
| `calibrate_once` | `false` | Exit the node automatically after one calibration cycle |
| `save_to_file` | `true` | Write the computed values to the `.env` file |
| `use_median` | `false` | Use the median instead of the mean for calibration values |
| `avoid_repeated_data` | `true` | Skip consecutive duplicate analog readings when computing the result |
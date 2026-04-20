from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=EnvironmentVariable('ROBOT_MODEL', default_value='rbvogui')
    )
    file_name_arg = DeclareLaunchArgument(
        'file_name',
        default_value=[LaunchConfiguration('robot_model'), '_params.env']
    )
    file_path_arg = DeclareLaunchArgument(
        'file_path',
        default_value='/home/robot/robot_settings/robot_params/base'
    )
    env_var_arg = DeclareLaunchArgument(
        'env_var',
        default_value='ROBOT_STEERING_CALIBRATION'
    )
    topic_sub_arg = DeclareLaunchArgument(
        'topic_sub',
        default_value='/robot/robotnik_io_controller/io'
    )
    calibration_duration_arg = DeclareLaunchArgument(
        'calibration_duration',
        default_value='60'
    )
    calibrate_once_arg = DeclareLaunchArgument(
        'calibrate_once',
        default_value='false'
    )
    save_to_file_arg = DeclareLaunchArgument(
        'save_to_file',
        default_value='true'
    )
    use_median_arg = DeclareLaunchArgument(
        'use_median',
        default_value='false'
    )
    avoid_repeated_data_arg = DeclareLaunchArgument(
        'avoid_repeated_data',
        default_value='true'
    )

    calibration_node = Node(
        package='rbvogui_wheels_calibration',
        executable='calibration_node',
        name='wheel_calibration',
        output='screen',
        parameters=[{
            'file_name': LaunchConfiguration('file_name'),
            'file_path': LaunchConfiguration('file_path'),
            'env_var': LaunchConfiguration('env_var'),
            'topic_sub': LaunchConfiguration('topic_sub'),
            'calibration_duration': LaunchConfiguration('calibration_duration'),
            'calibrate_once': LaunchConfiguration('calibrate_once'),
            'save_to_file': LaunchConfiguration('save_to_file'),
            'use_median': LaunchConfiguration('use_median'),
            'avoid_repeated_data': LaunchConfiguration('avoid_repeated_data'),
        }]
    )

    return LaunchDescription([
        robot_model_arg,
        file_name_arg,
        file_path_arg,
        env_var_arg,
        topic_sub_arg,
        calibration_duration_arg,
        calibrate_once_arg,
        save_to_file_arg,
        use_median_arg,
        avoid_repeated_data_arg,
        calibration_node,
    ])

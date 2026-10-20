from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('eng26_arm_driver'), 'config', 'driver.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='921600'),
        Node(
            package='eng26_arm_driver', executable='arm_driver_node',
            name='arm_driver', namespace=LaunchConfiguration('namespace'),
            output='screen', parameters=[LaunchConfiguration('params_file'),
                                         {'device': LaunchConfiguration('device'),
                                          'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int)}]),
    ])

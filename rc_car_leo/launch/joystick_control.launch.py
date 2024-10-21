from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('rc_car_leo'),
        'params',
        'vesc_config.yaml'
        )
    logitech_config = os.path.join(
        get_package_share_directory('rc_car_leo'),
        'params',
        'logitech.config.yaml'
        )
    return LaunchDescription ([
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[logitech_config]

        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
             parameters=[vesc_config]
             # eventually change parameters to use LaunchConfiguration, but I cannot get it to work that way right now.
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
             parameters=[vesc_config]
             # eventually change parameters to use LaunchConfiguration, but I cannot get it to work that way right now.
        ),
        Node(
            package='rc_car_leo',
            executable='ackermann_conversion',
            name='ackermann_conversion_node',
            output='screen'
        ),
         Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[vesc_config]
            # eventually change parameters to use LaunchConfiguration, but I cannot get it to work that way right now.
        )

    ])
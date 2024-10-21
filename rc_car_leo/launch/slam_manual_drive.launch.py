import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_visualization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_examples'), 'launch',
            'stereo.launch.py')])
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch',
            'test_launch.py')])
    )

    joystick_manual_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rc_car_leo'), 'launch',
            'joystick_control.launch.py')])
    )

    urdf_display_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rc_car_leo'), 'launch',
            'display.launch.py')])
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rc_car_leo'), 'launch',
            'robot_localization.launch.py')])
    )

    openrtk_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('openrtk_imu'), 'launch',
            'openrtk_launch.py')])
    )

    return LaunchDescription([
        localization_node,
        urdf_display_node,
        slam_node,
        joystick_manual_control_node,
        openrtk_node,
        camera_visualization_node,
    ])

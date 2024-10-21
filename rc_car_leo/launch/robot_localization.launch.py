import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()
    robot_localization_odom_config = os.path.join( 
            get_package_share_directory('rc_car_leo'),
            'params',
            'ekf_config_odom.yaml')
    
    robot_localization_map_config = os.path.join( 
            get_package_share_directory('rc_car_leo'),
            'params',
            'ekf_config_map.yaml')


    ekf_odom_node = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'robot_localization_ekf_node_odom',
        output = 'screen',
        parameters = [robot_localization_odom_config]
    )

    ekf_map_node = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'robot_localization_ekf_node_map',
        output = 'screen',
        parameters = [robot_localization_map_config]
    )
    
    ld.add_action(ekf_map_node)
    ld.add_action(ekf_odom_node)
    

    return ld
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()
    # openrtk_config = os.path.join( 
    #         get_package_share_directory('rc_car_leo'),
    #         'params',
    #         'ekf_config_odom.yaml')


    openrtk_node = Node(
        package = 'openrtk_imu',
        executable = 'openrtk_node',
        name = 'ros_openrtk_node',
        # parameters = [openrtk_config]
    )
    
    ld.add_action(openrtk_node)
    

    return ld
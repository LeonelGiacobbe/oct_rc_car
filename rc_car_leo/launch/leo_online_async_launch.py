import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import launch_ros.actions
import launch_ros.descriptions
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Joystick manual control node
    joystick_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource('/home/dev/leo_ws/src/Bridges_RC/rc_car_leo/launch/joystick_control.launch.py')
      )

    # Launch online async SLAM nodes
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='False',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value= '/home/dev/leo_ws/src/Bridges_RC/slam_toolbox/config/mapper_params_online_async.yaml',
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )


    # Launch stereo.launch.py


    default_rviz = os.path.join(get_package_share_directory('depthai_examples'),
                                'rviz', 'stereoPointCloud.rviz')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')
    

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix  = LaunchConfiguration('tf_prefix',   default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll   = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = '0.0')

    mode           = LaunchConfiguration('mode', default = 'depth')
    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = True)
    confidence     = LaunchConfiguration('confidence', default = 200)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 5)
    monoResolution = LaunchConfiguration('monoResolution',  default = '720p')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Contains the resolution of the Mono Cameras. Available resolutions are 800p, 720p & 400p for OAK-D & 480p for OAK-D-Lite.')

    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix' : tf_prefix,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items())

    stereo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_node',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'mode': mode},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'monoResolution': monoResolution}])


    metric_converter_node = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
            ],
            output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
            name='container2',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                ('intensity/image_rect', '/right/image_rect'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',)

    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])
    
    ld = LaunchDescription()

    # Slam toolbox actions

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)


    # Depthai Stereo actions


    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)
    
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)

    ld.add_action(stereo_node)
    ld.add_action(urdf_launch)

    ld.add_action(metric_converter_node)
    ld.add_action(point_cloud_node)
    ld.add_action(rviz_node)

    return ld
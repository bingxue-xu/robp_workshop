# hardware_ws/src/hardware_test/launch/hardware_checks_launch.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

def _validate_required_args(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    domain_id  = LaunchConfiguration('domain_id').perform(context)
    if not robot_name:
        raise RuntimeError("The 'robot_name' launch argument is required but was not provided.")
    if not domain_id:
        raise RuntimeError("The 'domain_id' launch argument is required but was not provided.")

def generate_launch_description():
    # --- 0. declare parameters ---
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        description='Name of the robot, e.g. Sneezy'
    )
    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        description='Domain ID of the robot, e.g. 0'
    )
    src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'src','hardware_test'))
    json_folder = os.path.join(src_dir, 'test_results')
    json_folder_arg = DeclareLaunchArgument(
        'json_folder',
        default_value=json_folder,
        description='Folder to store JSON results'
    )

    # odometry config handle
    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='1.0',
        description='Size of the square path in meters'
    )
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.2',
        description='Linear speed in m/s'
    )
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Angular speed in rad/s'
    )

    laps_per_direction_arg = DeclareLaunchArgument(
        'laps_per_direction',
        default_value='2',
        description='Number of laps to perform in each direction'
    )

    # config handle
    robot_name = LaunchConfiguration('robot_name')
    domain_id  = LaunchConfiguration('domain_id')
    json_folder= LaunchConfiguration('json_folder')
    square_size= LaunchConfiguration('square_size')
    speed      = LaunchConfiguration('speed')
    angular_speed = LaunchConfiguration('angular_speed')
    laps       = LaunchConfiguration('laps_per_direction')

    # --- 1. start tf  ---
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    static_base_link_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_to_laser',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link','laser']
    )

    # --- Odometry Node ---
    odometry_node = Node(
        package='odometry',
        executable='odometry',
        name='odometry_node',
        output='screen'
    )

    odometry_test_node = Node(
        package='hardware_test',
        executable='odometry_test',
        name='odometry_test',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'domain_id': domain_id},
            {'json_folder': json_folder},
            {'square_size': square_size},
            {'speed': speed},
            {'angular_speed': angular_speed},
            {'laps_per_direction': laps},
        ]
    )

    delayed_odometry_test_node = TimerAction(
        period=3.0,
        actions=[odometry_test_node]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('realsense2_description'),
                    'urdf',
                    'test_d435_camera.urdf.xacro'
                ]),
                ' use_nominal_extrinsics:=false'
            ]),
            'publish_frequency': 30.0,
        }]
    )

    cartesian_controller_node = Node(
        package='controller',
        executable='cartesian_controller',
        name='cartesian_controller',
        output='screen',
        parameters=[
            {'frequency': 20},
            {'wheel_base': 0.311},
            {'wheel_radius': 0.04921},
            {'ticks_per_revolution': 3072},
        ]
    )

    # --- Marker nodes ---
    marker_1_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_1',
        output='screen',
        arguments=['--x', '-1', '--y', '-1', '--z', '0.18', '--frame-id', 'map', '--child-frame-id', 'aruco/marker1']
    )
    marker_2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_2',
        output='screen',
        arguments=['--x', '2', '--y', '0', '--z', '0.18', '--yaw', '3.14159265359', '--frame-id', 'map', '--child-frame-id', 'aruco/marker2']
    )

    # --- 2. launch drivers ---
    # RealSense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'    
            )
        )
    )

    # # RPLidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar.launch.py'  
            )
        )
    )

    # phidgets_container 
    phidgets_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robp_launch'),
                'launch',
                'phidgets_launch.py'
            )
        ),
    )

    # --- 4. RViz ---
    rviz_cfg = os.path.join(
        get_package_share_directory('hardware_test'),
        'rviz',
        'odometry_test.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_odometry_test',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    # --- 5. LaunchDescription ---
    return LaunchDescription([
        # parameters
        robot_name_arg,
        domain_id_arg,
        json_folder_arg,
        square_size_arg,
        speed_arg,
        angular_speed_arg,
        laps_per_direction_arg,
        OpaqueFunction(function=_validate_required_args),
        # tf
        static_map_to_odom_node,
        static_base_link_to_laser_node,
        odometry_node,
        robot_state_publisher_node,
        cartesian_controller_node,

        # # marker nodes
        # marker_1_node,
        # marker_2_node,

        # # driver launch
        # # realsense_launch,
        # rplidar_launch,
        phidgets_launch,

        # RViz2
        rviz_node,
        delayed_odometry_test_node
    ])

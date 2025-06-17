# hardware_ws/src/hardware_test/launch/hardware_checks_launch.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    # config handle
    robot_name = LaunchConfiguration('robot_name')
    domain_id  = LaunchConfiguration('domain_id')
    json_folder= LaunchConfiguration('json_folder')

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
    odometry_node = Node(
        package='odometry',
        executable='odometry',
        name='odometry',
        output='screen',
        parameters=[
            {'frequency': 20},
            {'wheel_base': 0.311},
            {'wheel_radius': 0.04921},
            {'ticks_per_revolution': 3072},
        ]
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

    # USB Camera
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('usb_cam'),
                'launch',
                'camera.launch.py'
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
        launch_arguments={
            'robot_name': robot_name,
            'domain_id':  domain_id,
        }.items()
    )

    # --- 3. hardware checks ---
    realsense_check = Node(
        package='hardware_test',
        executable='realsense_check',
        name='realsense_check',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_name': robot_name},
            {'domain_id':  domain_id},
            # {'json_folder':json_folder},
            # {'topic_name':'/camera/camera/color/image_raw'},
            # {'timeout_s':   10.0},
        ]
    )

    rplidar_check = Node(
        package='hardware_test',
        executable='rplidar_check',
        name='rplidar_check',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_name':          robot_name},
            {'domain_id':           domain_id},
            {'json_folder':         json_folder},
            {'topic_name':          '/scan'},
            {'timeout_s':           10.0},
            {'point_count_threshold':100},
            {'range_threshold':     1.0},
        ]
    )
    phidgets_check = Node(
        package='hardware_test',
        executable='phidgets_check',
        name='phidgets_check',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_name': robot_name},
            {'domain_id':  domain_id},
        ]
    )
    usb_cam_check = Node(
        package='hardware_test',
        executable='usb_cam_check',
        name='usb_cam_check',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_name':  robot_name},
            {'domain_id':   domain_id},
            {'json_folder': json_folder},
            {'topic_name':  '/camera1/image_raw'},
            {'timeout_s':   10.0},
        ]
    )

    arm_check = Node(
        package='hardware_test',
        executable='arm_check',
        name='arm_check',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_name': robot_name},
            {'domain_id':  domain_id},
            {'json_folder': json_folder},
        ]
    )

    # --- 4. RViz ---
    rviz_cfg = os.path.join(
        get_package_share_directory('hardware_test'),
        'rviz',
        'hardware_test.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_hardware_test',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    # --- 5. LaunchDescription ---
    return LaunchDescription([
        # parameters
        robot_name_arg,
        domain_id_arg,
        json_folder_arg,
        OpaqueFunction(function=_validate_required_args),
        # tf
        static_map_to_odom_node,
        static_base_link_to_laser_node,
        odometry_node,
        robot_state_publisher_node,
        cartesian_controller_node,

        # # driver launch
        # realsense_launch,
        # rplidar_launch,
        # usb_cam_launch,
        # phidgets_launch,

        # # check nodes
        # realsense_check,
        # rplidar_check,
        # usb_cam_check,
        # phidgets_check,
        arm_check,

        # RViz2
        # rviz_node,
    ])

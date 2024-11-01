import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Basic configuration and paths
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map', default=os.path.join(get_package_share_directory('kwj_navigation2'), 'map', 'carto_map.yaml'))
    odom_topic = LaunchConfiguration('odom_topic', default='/odometry/filtered')

    # Paths for packages and files
    kwj_description_pkg = get_package_share_directory('kwj_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # URDF and map files
    urdf_file = os.path.join(kwj_description_pkg, 'urdf', 'kwj.urdf')
    param_file = os.path.join(get_package_share_directory('kwj_navigation2'), 'param', 'kwjbot.yaml')
    world_file = os.path.join(kwj_description_pkg, 'world', 'my_world2.world')

    # Load URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # Navigation2 parameter substitutions
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        # Immediate logging
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Declare launch arguments
        DeclareLaunchArgument('params_file', default_value=param_file, description='Path to the ROS2 parameters file'),
        DeclareLaunchArgument('map', default_value=map_file, description='Path to the map YAML file'),

        # EKF Node for localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{'odom0': odom_topic}]
        ),

        # Gazebo server and client for simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Navigation2 bringup with configured parameters, including map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_file  # Ensuring 'map' argument is passed explicitly
            }.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}],
            arguments=[urdf_file],
        ),

        # Static transforms
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0.065', '0', '0', '0', 'base_footprint', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0.11', '0', '0.18', '0', '0', '0', 'base_link', 'laser_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['-0.07', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0.193', '0.01', '-0.02', '0', '0', '0', 'base_link', 'caster_front_link']),
    ])


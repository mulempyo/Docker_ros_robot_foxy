import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Basic configuration and paths
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  
    params_file = LaunchConfiguration('params_file')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('kwj_navigation2'),
            'map',
            'carto_map.yaml'))
    odom_topic = LaunchConfiguration('odom_topic', default='/odometry/filtered')

    # Paths for packages and files
    kwj_description_pkg = get_package_share_directory('kwj_description')
    kwj_localization_pkg = get_package_share_directory('kwj_localization')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # URDF and map files
    urdf_file = os.path.join(kwj_description_pkg, 'urdf', 'kwj.urdf')
    param_file = os.path.join(get_package_share_directory('kwj_navigation2'), 'param', 'kwjbot.yaml')
    world_file = os.path.join(kwj_description_pkg, 'world', 'my_world2.world')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

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
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(kwj_localization_pkg, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )  
    
    cmd_vel_to_odom_node = Node(
    package='kwj_localization',
    executable='cmd_vel_to_odom',
    name='cmd_vel_to_odom',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('params_file', default_value=param_file, description='Path to the ROS2 parameters file'),
        DeclareLaunchArgument('map', default_value=map_dir, description='Path to the map YAML file'),
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        cmd_vel_to_odom_node,
        robot_localization_node,

        # Gazebo server and client for simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Navigation2 bringup with configured parameters, including map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_dir
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
      ])



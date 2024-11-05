from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
import xacro
import tempfile

def generate_launch_description():

    world_file_name = 'my_world2.world'
    urdf_file_name = 'kwjbot.urdf'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  
    params_file = LaunchConfiguration('params_file')
    map_dir = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    kwj_description_pkg = get_package_share_directory('kwj_description')
    kwj_localization_pkg = get_package_share_directory('kwj_localization')
    kwj_navigation2_pkg = get_package_share_directory('kwj_navigation2')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    rviz_config_file = os.path.join(kwj_navigation2_pkg, 'rviz', 'default.rviz')
    xacro_file = os.path.join(kwj_description_pkg, 'urdf', 'kwj.gazebo.xacro')
    param_file = os.path.join(kwj_navigation2_pkg, 'param', 'kwjbot.yaml')
    world = os.path.join(kwj_description_pkg, 'world', world_file_name)  
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    urdf = os.path.join(kwj_description_pkg, 'urdf', urdf_file_name)

    try:
        gazebo_xacro = xacro.process_file(xacro_file)
        gazebo_urdf = gazebo_xacro.toxml()
        temp_gazebo_urdf = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
        temp_gazebo_urdf.write(gazebo_urdf.encode('utf-8'))
        temp_gazebo_urdf.close()
    except Exception as e:
        print(f"Failed to process xacro file: {xacro_file}")
        print(e)
        return LaunchDescription([])

    default_map_path = PathJoinSubstitution([FindPackageShare('kwj_navigation2'), 'map', 'turtlebot3_world.yaml'])

    # Launch Arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Whether to autostart the Nav2 stack'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=param_file,
        description='Full path to the ROS2 parameters file to use'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf]
    )
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(kwj_localization_pkg, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    cmd_vel_to_odom_node = Node(
        package='kwj_localization',
        executable='cmd_vel_to_odom',
        name='cmd_vel_to_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gazebo 
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-file', temp_gazebo_urdf.name, '-entity', 'KWJBot']
    )
    spawn_entity_with_delay = TimerAction(
        period=3.0, 
        actions=[spawn_entity]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)  
    )

    # Navigation Bringup 
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav2_launch_file_dir, 'bringup_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_dir,
            'autostart': autostart
        }.items()
    )

    # Launch Description 
    ld = LaunchDescription()

    # Declare Actions
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add Nodes and Launch Files
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_localization_node)
    ld.add_action(cmd_vel_to_odom_node)
    ld.add_action(nav2_bringup)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_entity_with_delay) 
    ld.add_action(rviz_node)

    return ld


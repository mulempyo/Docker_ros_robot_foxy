from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    kwj_description_pkg = get_package_share_directory('kwj_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    kwj_localization_pkg = get_package_share_directory('kwj_localization')  

    # Use xacro to process the urdf.xacro file and get the robot description
    robot_description = Command(['ros2 run xacro xacro ', kwj_description_pkg, '/urdf/kwj.urdf.xacro'])

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'world_name',
            default_value=[kwj_description_pkg, '/world/my_world.world'],
            description='World file to load'
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start simulation paused'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run headless'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': LaunchConfiguration('gui')}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world_name'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'headless': LaunchConfiguration('headless'),
                'debug': LaunchConfiguration('debug'),
            }.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'KwjBot'],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_sensor_fusion_node',
            parameters=[os.path.join(kwj_localization_pkg, 'config', 'ekf.yaml')]  
        ),

        # Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.065', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.11', '0', '0.18', '0', '0', '0', 'base_link', 'laser_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.07', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.193', '0.01', '-0.02', '0', '0', '0', 'base_link', 'caster_front_link']
        ),
    ])


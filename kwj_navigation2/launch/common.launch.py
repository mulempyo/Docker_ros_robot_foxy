# common.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    kwj_description_pkg = get_package_share_directory('kwj_description')
    
    # Use xacro to process the urdf.xacro file and get the robot description
    urdf_file = os.path.join(kwj_description_pkg, 'urdf', 'kwj.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start simulation paused'
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
        DeclareLaunchArgument(
            'world_name',
            default_value=os.path.join(kwj_description_pkg, 'world', 'my_world.world'),
            description='World file to load'
        ),
    ])


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Define package directories
    kwj_bringup_dir = get_package_share_directory('kwj_bringup')
    kwj_description_dir = get_package_share_directory('kwj_description')
    kwj_localization_dir = get_package_share_directory('kwj_localization')
    kwj_slam_dir = get_package_share_directory('kwj_slam')
    
    return LaunchDescription([
        
        # Include kwj_bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([kwj_bringup_dir, '/launch/kwj_bringup.launch.py'])
        ),

        # Include URDF launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([kwj_description_dir, '/launch/urdf.launch.py'])
        ),

        # cmd_vel to odom node
        Node(
            package='kwj_localization',
            executable='cmd_vel_to_odom',
            name='cmd_vel_to_odom',
            output='screen'
        ),

        # EKF localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_se_odom',
            parameters=[kwj_localization_dir + '/config/ekf.yaml'],
            output='screen',
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', kwj_localization_dir + '/rviz/odom_test.rviz'],
            output='screen',
        ),

        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[],
            arguments=[
                '-configuration_directory', kwj_slam_dir + '/config',
                '-configuration_basename', 'kwj_lidar.lua'
            ],
            remappings=[
                ('scan', 'scan'),
            ]
        ),

        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}]
        ),
    ])


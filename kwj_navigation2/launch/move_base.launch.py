from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    kwj_bringup_pkg = get_package_share_directory('kwj_bringup')
    kwj_description_pkg = get_package_share_directory('kwj_description')
    kwj_navigation_pkg = get_package_share_directory('kwj_navigation')
    
    return LaunchDescription([
        # Declare odom_topic argument
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odometry/filtered',
            description='Odometry topic name'
        ),
        
        # Include bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kwj_bringup_pkg + '/launch/kwj_bringup.launch.py')
        ),
        
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kwj_description_pkg + '/launch/gazebo.launch.py')
        ),
        
        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': kwj_navigation_pkg + '/map/carto_map.yaml'
            }]
        ),

        # Odom publisher node
        Node(
            package='kwj_localization',
            executable='odom_pub',
            name='odom_pub',
            output='screen'
        ),

        # EKF Localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_sensor_fusion_node',
            output='screen',
            parameters=[kwj_navigation_pkg + '/config/ekf.yaml']
        ),

        # Move base node (replace move_base with ROS2 Navigation stack equivalent)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='move_base',
            output='screen',
            parameters=[
                kwj_navigation_pkg + '/param/costmap_common_params.yaml',
                kwj_navigation_pkg + '/param/local_costmap_params.yaml',
                kwj_navigation_pkg + '/param/global_costmap_params.yaml',
                kwj_navigation_pkg + '/param/move_base_params.yaml',
            ]
        ),

        # Rviz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', kwj_navigation_pkg + '/rviz/rviz.rviz']
        ),
    ])


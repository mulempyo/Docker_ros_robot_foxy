from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define package directories
    kwj_description_dir = get_package_share_directory('kwj_description')
    kwj_localization_dir = get_package_share_directory('kwj_localization')

    return LaunchDescription([
        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([kwj_description_dir, '/launch/gazebo.launch.py'])
        ),

        # cmd_vel to odom node
        Node(
            package='kwj_localization',
            executable='cmd_vel_to_odom',
            name='cmd_vel_to_odom',
            output='screen'
        ),

        # EKF Localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_se_odom',
            parameters=[kwj_localization_dir + '/config/ekf.yaml'],
            output='screen'
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', kwj_localization_dir + '/rviz/odom_test.rviz'],
            output='screen',
        ),

        # RTAB-Map node
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_depth': True,
                'subscribe_scan': False,
                'subscribe_stereo': True,
                'frame_id': 'base_link',
                'approx_sync': True,
                'Mem/IncrementalMemory': True,
                'RGBD/OptimizeFromGraphEnd': True,
            }]
        ),

        # RTAB-Map Visualizer node
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen',
            parameters=[{
                'subscribe_depth': True,
                'subscribe_stereo': True,
                'frame_id': 'base_link',
            }]
        ),
    ])


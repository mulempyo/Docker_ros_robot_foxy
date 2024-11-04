from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import tempfile

def generate_launch_description():
   
    package_name = 'kwj_description'
    gazebo_ros_pkg = 'gazebo_ros'
    navigation2_pkg = 'kwj_navigation2'

  
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'kwjbot.urdf')
    xacro_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'kwj.gazebo.xacro')
    rviz_config_file = os.path.join(get_package_share_directory(navigation2_pkg), 'rviz', 'default.rviz')

    
    try:
        with open(urdf_file, 'r') as infp:
            robot_description_content = infp.read()
    except FileNotFoundError:
        print(f"Don't find URDF file: {urdf_file}")
        return LaunchDescription([])

    robot_description = {'robot_description': robot_description_content}

   
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(gazebo_ros_pkg),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'verbose': 'true'}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-file', temp_gazebo_urdf.name,
                   '-entity', 'KWJBot'],
    )

    spawn_entity_with_delay = TimerAction(
        period=5.0,  
        actions=[spawn_entity]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_with_delay,
        rviz_node
    ])


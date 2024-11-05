from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import tempfile
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
   
    navigation2_pkg = 'kwj_navigation2'
    world_file_name = 'my_world2.world'
    urdf_file_name = 'kwjbot.urdf'
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    kwj_description_pkg = get_package_share_directory('kwj_description')
  
    xacro_file = os.path.join(kwj_description_pkg, 'urdf', 'kwj.gazebo.xacro')
    rviz_config_file = os.path.join(get_package_share_directory(navigation2_pkg), 'rviz', 'default.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = os.path.join(get_package_share_directory('kwj_description'), 'world', world_file_name)

    urdf = os.path.join(
        get_package_share_directory('kwj_description'),
        'urdf',
        urdf_file_name)

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
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )  
    
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        robot_state_publisher_node,
        gazebo_client,
        gazebo_server,
        spawn_entity_with_delay,
        joint_state_publisher_node,
        rviz_node
    ])


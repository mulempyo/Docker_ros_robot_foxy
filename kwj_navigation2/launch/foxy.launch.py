from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import tempfile

def generate_launch_description():
   
    # Launch Configuration 정의
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  
    params_file = LaunchConfiguration('params_file')
    map_dir = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_ros_pkg = 'gazebo_ros'  # 패키지 이름 수정

    # 패키지 경로 설정
    kwj_description_pkg = get_package_share_directory('kwj_description')
    kwj_localization_pkg = get_package_share_directory('kwj_localization')
    kwj_navigation2_pkg = get_package_share_directory('kwj_navigation2')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    rviz_config_file = os.path.join(kwj_navigation2_pkg, 'rviz', 'default.rviz')
    
    # 파일 경로 설정
    urdf_file = os.path.join(kwj_description_pkg, 'urdf', 'kwjbot.urdf')
    xacro_file = os.path.join(kwj_description_pkg, 'urdf', 'kwj.gazebo.xacro')
    param_file = os.path.join(kwj_navigation2_pkg, 'param', 'kwjbot.yaml')
    world_file = os.path.join(kwj_description_pkg, 'world', 'my_world2.world')
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    # URDF 파일 읽기
    try:
        with open(urdf_file, 'r') as infp:
            robot_description_content = infp.read()
    except FileNotFoundError:
        print(f"Don't find urdf file: {urdf_file}")
        return LaunchDescription([])
        
    # Xacro 파일 처리 및 임시 URDF 파일 생성
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

    # robot_description 설정
    robot_description = {'robot_description': robot_description_content}

    # Launch Argument 선언
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
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
    
    # 노드 정의
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(kwj_localization_pkg, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )  
    
    cmd_vel_to_odom_node = Node(
        package='kwj_localization',
        executable='cmd_vel_to_odom',
        name='cmd_vel_to_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Nav2 Bringup Launch 포함
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav2_launch_file_dir, 'bringup_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_dir,
            'autostart': autostart
        }.items()
    )
    
    # Gazebo Launch 포함
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(gazebo_ros_pkg),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'verbose': 'true',
            'world': world_file  # world 파일 경로 추가
        }.items(),
    )
    
    # Gazebo에 로봇 스폰하는 노드 정의
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-file', temp_gazebo_urdf.name, '-entity', 'KWJBot']
    )

    # Gazebo가 완전히 시작된 후 로봇을 스폰하기 위해 지연 시간 추가
    spawn_entity_with_delay = TimerAction(
        period=5.0,  # 5초 지연
        actions=[spawn_entity]
    )
   
    # RViz 노드 정의
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)  # RViz 실행 여부에 따른 조건 추가
    )
   
    # Launch Description에 액션 추가
    ld = LaunchDescription()

    # Launch Argument 추가
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # 노드 및 Launch 파일 추가
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_localization_node)
    ld.add_action(cmd_vel_to_odom_node)
    ld.add_action(nav2_bringup)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity_with_delay)  # 지연 후 스폰
    ld.add_action(rviz_node)
    
    return ld


from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kwj_description'  # 실제 패키지 이름으로 변경
    xacro_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'kwj.gazebo.xacro')
    
    # Xacro 파일을 URDF로 변환
    robot_description_content = None
    try:
        with open(xacro_file, 'r') as infp:
            robot_description_content = infp.read()
    except FileNotFoundError:
        print(f"Xacro 파일을 찾을 수 없습니다: {xacro_file}")
        return LaunchDescription([])
    
    robot_description = {'robot_description': robot_description_content}
    
    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Gazebo 실행
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        name='gazebo',
        output='screen',
        arguments=['-s', 'libgazebo_ros_factory.so']
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_node
    ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_outdoor_example = get_package_share_directory('nav2_outdoor_example')

    # 기존 outdoor_nav.launch.py의 내용을 대부분 가져옵니다.
    use_sim_time = True
    world_path = os.path.join(pkg_nav2_outdoor_example, 'worlds', 'outdoor.world')
    robot_launch_path = os.path.join(pkg_nav2_outdoor_example, 'launch', 'robot.launch.py')
    nav_launch_path = os.path.join(pkg_nav2_outdoor_example, 'launch', 'nav.launch.py')

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 로봇 모델 및 robot_localization 실행
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={'use_sim_time': str(use_sim_time)}.items()
    )

    # Nav2 실행
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={'use_sim_time': str(use_sim_time)}.items()
    )

    # --- SLAM Toolbox 노드 추가 ---
    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          os.path.join(pkg_nav2_outdoor_example, 'config', 'slam_online.yaml'),
          {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        gazebo,
        robot,
        nav,
        start_slam_toolbox_node # SLAM 노드를 마지막에 추가
    ])
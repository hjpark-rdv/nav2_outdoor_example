#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_outdoor_example')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/simulation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items() # 이 부분 추가
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/visualization.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items() # 이 부분 추가
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/localization.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items() # 이 부분 추가
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/navigation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items() # 이 부분 추가
    )
# --- SLAM 런치 파일 포함시키기 (아래 코드 추가) ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/slam.launch.py')),
        # use_sim_time 파라미터를 slam.launch.py에 전달
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return LaunchDescription(
        [
            simulation,
            visualization,
            localization,
            navigation,
            slam,
        ]
    )



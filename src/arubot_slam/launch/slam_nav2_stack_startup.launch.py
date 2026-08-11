import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    pkg_name = "arubot_slam"

    
    # SLAM Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                "slam_online_async_launch.py"
            )), 
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(get_package_share_directory(pkg_name), 'config', 'map_param.yaml')
            }.items()
    )

    # Nav2 Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                'navigation_launch.py'
            )),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(get_package_share_directory(pkg_name), 'config', 'nav2.yaml')
        }.items()
    )

    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)

    return ld
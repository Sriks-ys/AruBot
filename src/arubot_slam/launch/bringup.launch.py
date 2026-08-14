import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_id = LaunchConfiguration('package_id')

    package_arguement = DeclareLaunchArgument(
        'package_id',
        default_value='10',
        description="Robot will look for package with this id"
    )
    sllidar_pkg_dir = get_package_share_directory('sllidar_ros2')
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg_dir, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/lidar'
        }.items()
    )


    mecanum_pkg_dir = get_package_share_directory('mecanum_robot_hardware')
    mecanum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg_dir, 'launch', 'mecanumbot.launch.py')
        )
    )


    tf_odometry_relay_node = Node(
        package='miscellaneous',
        executable='tf_odometry_relay',
        name='tf_odometry_relay',
        output='screen'
    )

    network_mqtt = Node(
        package='network',
        executable='mqtt_bridge',
        name="mqtt_bridge",
        output='screen'
    )

    goal_manager = Node(
        package = 'goal_managers',
        executable = 'goal_manager',
        name = 'GoalManager',
        output = 'screen'
    )

    servo_bridge = Node(
        package = 'miscellaneous',
        executable = 'servo_hardware_controller',
        name = 'servo_bridge',
        output = 'screen'
    )

    camera_node = Node(
        package = "goal_managers",
        executable = "aruco_detector",
        name = "camera_node",
        output = "screen",
        parameters = [
            {
                "target_id": target_id
            }
        ]
    )

    controller_node = Node(
        package = "goal_managers",
        executable = "retreat_node",
        name = "retreater",
    )

    door_node = Node(
        package = "network",
        executable = "door_client",
        name = "door_client"
    )


    delayed_mecanum = TimerAction(
        period=5.0,
        actions=[mecanum_launch, servo_bridge, camera_node]
    )

    delayed_relay = TimerAction(
        period=11.0,
        actions=[tf_odometry_relay_node, network_mqtt, goal_manager, door_node]
    )

    return LaunchDescription([
        package_arguement,
        sllidar_launch,
        delayed_mecanum,
        delayed_relay, 
        controller_node
    ])

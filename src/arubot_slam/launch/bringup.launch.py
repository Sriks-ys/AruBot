import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Declare Launch Arguments
    # -------------------------------------------------------------------------
    # Allows overriding the serial port from the CLI (e.g., serial_port:=/dev/ttyUSB1)
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB2',
        description='Specifies the serial port for the SLLiDAR sensor.'
    )

    # -------------------------------------------------------------------------
    # 2. Include SLLiDAR (Laser Scanner) Launch File
    #    NOTE: This is launched immediately, with no delay, so it gets first
    #    access to system resources during its serial handshake / init phase.
    # -------------------------------------------------------------------------
    sllidar_pkg_dir = get_package_share_directory('sllidar_ros2')
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg_dir, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )

    # -------------------------------------------------------------------------
    # 3. Include Mecanum Robot Hardware Base Launch File
    # -------------------------------------------------------------------------
    mecanum_pkg_dir = get_package_share_directory('mecanum_robot_hardware')
    mecanum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg_dir, 'launch', 'mecanumbot.launch.py')
        )
    )

    # -------------------------------------------------------------------------
    # 4. Define TF / Odometry Relay Node
    # -------------------------------------------------------------------------
    tf_odometry_relay_node = Node(
        package='miscellaneous',
        executable='tf_odometry_relay',
        name='tf_odometry_relay',
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 4.5 Stagger startup so that no two serial-port-dependent nodes try to
    #     open their ports at the same instant. Both the lidar AND the
    #     mecanum base (rear arduino) talk over serial, and launching them
    #     simultaneously has been observed to make ONE of them fail to open
    #     its port (sometimes the lidar times out, sometimes the arduino
    #     fails to open). Staggering each serial-dependent stage gives each
    #     one a clear window to complete its handshake before the next one
    #     starts.
    #
    #     Stage 1 (t=0s):  sllidar_launch    (serial: lidar)
    #     Stage 2 (t=5s):  mecanum_launch     (serial: rear arduino)
    #     Stage 3 (t=10s): tf relay           (no serial, just needs the
    #                      mecanum topic to already exist)
    # -------------------------------------------------------------------------
    delayed_mecanum = TimerAction(
        period=5.0,
        actions=[mecanum_launch]
    )

    delayed_relay = TimerAction(
        period=10.0,
        actions=[tf_odometry_relay_node]
    )

    # -------------------------------------------------------------------------
    # 5. Assemble and Return the Launch Description
    # -------------------------------------------------------------------------
    return LaunchDescription([
        serial_port_arg,
        sllidar_launch,
        delayed_mecanum,
        delayed_relay
    ])

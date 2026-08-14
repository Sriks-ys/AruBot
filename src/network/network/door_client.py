import socket
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt8


class DoorClientNode(Node):
    def __init__(self):
        super().__init__('door_client')

        self.door_ip = None
        self.door_port = 50000

        self.monitoring_door = False
        self.latched_distance = None

        self.front_distance = float('inf')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.door_cmd_sub = self.create_subscription(
            String,
            '/door_cmd',
            self.door_cmd_callback,
            10
        )

        self.door_feedback_pub = self.create_publisher(
            UInt8,
            '/door_feedback',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Door Client Node started. Waiting for /door_cmd...'
        )

    def scan_callback(self, msg):
        """
        Get the LiDAR reading closest to angle 0 degrees.
        """

        if not msg.ranges:
            self.front_distance = float('inf')
            return

        # Find the scan index closest to angle 0 radians
        zero_index = round(
            (0.0 - msg.angle_min) / msg.angle_increment
        )

        if zero_index < 0 or zero_index >= len(msg.ranges):
            self.front_distance = float('inf')
            return

        distance = msg.ranges[zero_index]

        # Validate measurement
        if (
            math.isnan(distance)
            or math.isinf(distance)
            or distance < msg.range_min
            or distance > msg.range_max
        ):
            self.front_distance = float('inf')
            return

        self.front_distance = distance


    def door_cmd_callback(self, msg):
        command = msg.data
        if command[0] == '0':

            if self.monitoring_door:
                self.get_logger().warn(
                    'Already monitoring an OPEN command.'
                )
                return

            if math.isinf(self.front_distance):
                self.get_logger().warn(
                    'Cannot OPEN: invalid LiDAR reading at angle 0.'
                )
                return


            self.latched_distance = self.front_distance

            self.get_logger().info(
                f'OPEN received. '
                f'Latched distance at angle 0: '
                f'{self.latched_distance:.3f} m'
            )

            if self.send_door_command(command[2:],'open'):

                self.monitoring_door = True

                self.get_logger().info(
                    'OPEN command sent. '
                    'Now monitoring LiDAR for door movement...'
                )

            else:
                self.get_logger().error(
                    'Failed to send OPEN command.'
                )

                self.latched_distance = None
                self.monitoring_door = False

        elif command[0] == '1':

            self.get_logger().info(
                'CLOSE received. Sending CLOSE command...'
            )

            # CLOSE does not start LiDAR monitoring
            if self.send_door_command(command[2:], 'close'):
                self.get_logger().info(
                    'CLOSE command sent successfully.'
                )
            else:
                self.get_logger().error(
                    'Failed to send CLOSE command.'
                )

        else:
            self.get_logger().warn(
                f'Unknown command: "{msg.data}"'
            )

    def control_loop(self):
        if not self.monitoring_door:
            return

        if self.latched_distance is None:
            return

        if math.isinf(self.front_distance):
            return

        if self.front_distance - self.latched_distance > 1.0: # Use 2.5 for ISA Test space

            self.get_logger().info(
                f'Door opened/moved away. '
                f'Latched: {self.latched_distance:.3f} m, '
                f'Current: {self.front_distance:.3f} m'
            )

            feedback = UInt8()
            feedback.data = 0

            self.door_feedback_pub.publish(feedback)

            self.get_logger().info(
                'Published /door_feedback: 0'
            )

            self.monitoring_door = False
            self.latched_distance = None


    def send_door_command(self, IP, command):
        self.get_logger().info(f"Sending to IP: {IP}")
        try:
            with socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            ) as sock:

                sock.settimeout(2.0)

                sock.sendto(
                    command.encode('utf-8'),
                    (IP, self.door_port)
                )
            return True

        except Exception as e:
            self.get_logger().error(
                f'Failed to send UDP command "{command}": {e}'
            )
            return False




def main(args=None):
    rclpy.init(args=args)

    node = DoorClientNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

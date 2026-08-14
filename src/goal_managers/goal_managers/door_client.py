import socket
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt8


class DoorClientNode(Node):
    def __init__(self):
        super().__init__('door_client')

        # --------------------------------------------------
        # Door controller UDP configuration
        # --------------------------------------------------
        self.door_ip = '144.32.165.89'
        self.door_port = 50000

        # --------------------------------------------------
        # Door state
        # --------------------------------------------------
        self.monitoring_door = False
        self.latched_distance = None

        # Latest LiDAR distance at angle 0
        self.front_distance = float('inf')

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
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

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------
        self.door_feedback_pub = self.create_publisher(
            UInt8,
            '/door_feedback',
            10
        )

        # --------------------------------------------------
        # Control loop
        # --------------------------------------------------
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Door Client Node started. Waiting for /door_cmd...'
        )

    # ======================================================
    # LiDAR callback
    # ======================================================

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

    # ======================================================
    # Door command callback
    # ======================================================

    def door_cmd_callback(self, msg):
        """
        Handle commands from /door_cmd.

        OPEN:
            1. Latch current LiDAR distance at angle 0.
            2. Send UDP 'open'.
            3. Start monitoring LiDAR.

        CLOSE:
            Send UDP 'close'.
            No LiDAR monitoring.
        """

        command = msg.data.strip().upper()

        # --------------------------------------------------
        # OPEN
        # --------------------------------------------------
        if command == 'OPEN':

            # Ignore another OPEN while already monitoring
            if self.monitoring_door:
                self.get_logger().warn(
                    'Already monitoring an OPEN command.'
                )
                return

            # Need a valid LiDAR reading
            if math.isinf(self.front_distance):
                self.get_logger().warn(
                    'Cannot OPEN: invalid LiDAR reading at angle 0.'
                )
                return

            # Latch the distance BEFORE sending OPEN
            self.latched_distance = self.front_distance

            self.get_logger().info(
                f'OPEN received. '
                f'Latched distance at angle 0: '
                f'{self.latched_distance:.3f} m'
            )

            # Send OPEN command
            if self.send_door_command('open'):

                # Only start monitoring AFTER OPEN was sent
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

        # --------------------------------------------------
        # CLOSE
        # --------------------------------------------------
        elif command == 'CLOSE':

            self.get_logger().info(
                'CLOSE received. Sending CLOSE command...'
            )

            # CLOSE does not start LiDAR monitoring
            if self.send_door_command('close'):
                self.get_logger().info(
                    'CLOSE command sent successfully.'
                )
            else:
                self.get_logger().error(
                    'Failed to send CLOSE command.'
                )

        # --------------------------------------------------
        # Unknown command
        # --------------------------------------------------
        else:
            self.get_logger().warn(
                f'Unknown command: "{msg.data}"'
            )

    # ======================================================
    # Monitoring loop
    # ======================================================

    def control_loop(self):
        """
        Only runs the door-opening detection after an OPEN
        command has been successfully sent.

        When the current distance becomes greater than the
        latched distance, publish UInt8(0).
        """

        if not self.monitoring_door:
            return

        if self.latched_distance is None:
            return

        if math.isinf(self.front_distance):
            return

        # --------------------------------------------------
        # Door has moved away
        # --------------------------------------------------
        if self.front_distance - self.latched_distance > 2.5:

            self.get_logger().info(
                f'Door opened/moved away. '
                f'Latched: {self.latched_distance:.3f} m, '
                f'Current: {self.front_distance:.3f} m'
            )

            # Publish feedback = 0
            feedback = UInt8()
            feedback.data = 0

            self.door_feedback_pub.publish(feedback)

            self.get_logger().info(
                'Published /door_feedback: 0'
            )

            # Stop monitoring
            self.monitoring_door = False
            self.latched_distance = None

    # ======================================================
    # UDP command
    # ======================================================

    def send_door_command(self, command):
        """
        Send a UDP command to the door controller.

        command:
            'open'
            'close'
        """

        try:
            with socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            ) as sock:

                sock.settimeout(2.0)

                sock.sendto(
                    command.encode('utf-8'),
                    (self.door_ip, self.door_port)
                )

            return True

        except Exception as e:
            self.get_logger().error(
                f'Failed to send UDP command "{command}": {e}'
            )
            return False


# ==========================================================
# Main
# ==========================================================

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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose
from std_msgs.msg import UInt8
import tf2_ros
import math


class LinearCtrlNode(Node):
    def __init__(self):
        super().__init__('linear_ctrl_node')

        self.cmd_vel_pub_ = self.create_publisher(
            TwistStamped,
            '/mecanum_base_controller/reference',
            10
        )

        self.controller_cmd_sub = self.create_subscription(
            Pose,
            '/controller_cmd',
            self.controller_cmd_callback,
            10
        )

        self.controller_cmd_pub_ = self.create_publisher(
            UInt8,
            '/controller_feedback',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.timer_period = 0.1
        self.max_speed = 0.08
        self.min_speed = 0.02
        self.position_tolerance = 0.01

        self.target_x = None
        self.target_y = None

        self.target_yaw = 0.0
        self.orientation_kp = 0.5
        self.max_angular_speed = 0.3
        self.orientation_tolerance = math.radians(2.0)

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        self.get_logger().info('Linear position controller started')

    def controller_cmd_callback(self, msg: Pose):
        self.target_x = msg.position.x
        self.target_y = msg.position.y

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        self.get_logger().info(
            f'New target: x={self.target_x:.3f}, '
            f'y={self.target_y:.3f}'
        )

    def timer_callback(self):
        if self.target_x is None or self.target_y is None:
            self.timer.cancel()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            return

        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y

        error_x = self.target_x - current_x
        error_y = self.target_y - current_y

        distance = math.hypot(error_x, error_y)

        current_yaw = self.get_yaw(transform.transform.rotation)

        yaw_error = self.normalize_angle(
            self.target_yaw - current_yaw
        )

        if (
            distance <= self.position_tolerance
            and abs(yaw_error) <= self.orientation_tolerance
        ):
            self.timer.cancel()
            self.stop()
            self.controller_cmd_pub_.publish(UInt8(data=0))

            self.get_logger().info(
                f'Target reached: x={current_x:.3f}, '
                f'y={current_y:.3f}, '
                f'yaw={math.degrees(current_yaw):.1f}'
            )

            self.target_x = None
            self.target_y = None
            return

        if distance > 0.0:
            direction_x = error_x / distance
            direction_y = error_y / distance

            speed = min(
                self.max_speed,
                max(self.min_speed, distance)
            )

            map_vx = direction_x * speed
            map_vy = direction_y * speed
        else:
            map_vx = 0.0
            map_vy = 0.0

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        base_vx = cos_yaw * map_vx + sin_yaw * map_vy
        base_vy = -sin_yaw * map_vx + cos_yaw * map_vy

        angular_z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                self.orientation_kp * yaw_error
            )
        )

        self.publish_velocity(
            linear_x=base_vx,
            linear_y=base_vy,
            angular_z=angular_z
        )

    def publish_velocity(
        self,
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0
    ):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular_z

        self.cmd_vel_pub_.publish(msg)

    def stop(self):
        self.publish_velocity()

    @staticmethod
    def get_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )


def main(args=None):
    rclpy.init(args=args)

    node = LinearCtrlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

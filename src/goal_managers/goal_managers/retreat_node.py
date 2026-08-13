import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, UInt8

class RetreatNode(Node):
    def __init__(self):
        super().__init__('retreat_node')

        # 速度控制发布者
        self.cmd_vel_pub_ = self.create_publisher(
            TwistStamped, '/mecanum_base_controller/reference', 10)

        self.controller_cmd_sub = self.create_subscription(String, "/controller_cmd", self.controller_cmd_callback, 10)
        self.controller_cmd_pub = self.create_publisher(UInt8, "/controller_feedback", 10)
        self.get_logger().info('Retreat Node started! Executing 40cm backup sequence...')

        self.elapsed_time = 0.0
        self.timer_period = 0.1  # 10Hz 发布频率

    def timer_callback(self):
        # 倒车 40cm：速度 -0.20 m/s，持续 2.0 秒 (-0.20 * 2 = -0.40m)
        if self.elapsed_time < 12.5:
            self.publish_velocity(linear_x=-0.08)
            self.elapsed_time += self.timer_period
        else:
            self.timer.cancel()
            self.stop()
            self.controller_cmd_pub.publish(UInt8(data = 0))
            self.get_logger().info('Retreat 40cm finished! Stopping node.')

    def publish_velocity(self, linear_x=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)
    
    def controller_cmd_callback(self, msg: String):
        if msg.data == "retreat":
            self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def stop(self):
        self.publish_velocity(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RetreatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

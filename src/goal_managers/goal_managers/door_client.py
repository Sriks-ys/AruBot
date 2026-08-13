import socket
import rclpy
from rclpy.node import Node


class DoorClientNode(Node):
    def __init__(self):
        super().__init__('door_client')

        # 门的网络信息 (对应你屏幕上的 IP 和 50000 端口)
        self.door_ip = '144.32.70.247'
        self.door_port = 50000

        self.get_logger().info('Door Client initialized. Preparing to send open command...')

        # 节点启动后立即触发发送开门指令
        self.send_open_command()

    def send_open_command(self):
        try:
            # 创建 TCP Socket 并发送 b'open'
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(3.0)  # 设置 3 秒超时
                s.connect((self.door_ip, self.door_port))
                s.sendall(b'open')

                self.get_logger().info(f"Successfully sent 'open' command to {self.door_ip}:{self.door_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to send open command to door: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DoorClientNode()
    # 执行完发消息后自动关闭节点
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

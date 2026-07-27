import json 

import rclpy
from rclpy.node import Node

import paho.mqtt.client as mqtt

BROKER_IP = "localhost"
MQTT_TOPIC = "Target_location"

class mqtt_bridge(Node):
    def __init__(self):
        super().__init__("mqtt_bridge")

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(BROKER_IP, 1883)
        self.client.loop_start()

        self.get_logger().info("Waiting for Package location from drone")

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(MQTT_TOPIC)
        self.get_logger().info("Ground Station connected")

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.get_logger().info(f"Data: {data}")

        self.client.loop_stop()
        self.client.disconnect()
        rclpy.shutdown()

def main():
    rclpy.init()

    node = mqtt_bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
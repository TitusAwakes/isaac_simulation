import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class BatteryLevelSubscriber(Node):
    def __init__(self):
        super().__init__("battery_level_subscriber")
        self.subscription = self.create_subscription(
            Float32MultiArray, "battery_levels", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        battery_levels = msg.data
        self.get_logger().info(f"Received battery levels: {battery_levels}")
        # Forward it to the control node


def main(args=None):
    rclpy.init(args=args)
    node = BatteryLevelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

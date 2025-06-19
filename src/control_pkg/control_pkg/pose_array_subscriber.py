import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class PoseArraySubscriber(Node):

    def __init__(self):
        super().__init__('pose_array_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            'pose_array_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: PoseArray):
        self.get_logger().info(f'Received {len(msg.poses)} poses:')
        for i, pose in enumerate(msg.poses):
            self.get_logger().info(
                f'Pose {i}: Position(x={pose.position.x}, y={pose.position.y}, z={pose.position.z}), '
                f'Orientation(x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w})'
            )
            
        # Forward it to the control node, probably build a device pose table. Probably can run this only once.

def main(args=None):
    rclpy.init(args=args)
    node = PoseArraySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray


class TagLogger(Node):
    def __init__(self):
        super().__init__('tag_logger')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        self.detected_tags = set()

    def listener_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]  # Assuming single tag per detection
            if tag_id not in self.detected_tags:
                self.detected_tags.add(tag_id)
                self.get_logger().info(f"Tag detected: ID {tag_id}, Pose: {detection.pose.pose.pose}")


def main(args=None):
    rclpy.init(args=args)
    tag_logger = TagLogger()
    rclpy.spin(tag_logger)
    tag_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

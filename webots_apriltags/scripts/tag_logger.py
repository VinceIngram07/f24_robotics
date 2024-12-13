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
            try:
                tag_id = detection.id  # Check if `id` is valid
                center = detection.centre  # Adjust based on actual field names
                corners = detection.corners  # Example for logging corners
                if tag_id not in self.detected_tags:
                    self.detected_tags.add(tag_id)
                    self.get_logger().info(
                        f"Tag detected: ID {tag_id}, Center: ({center.x}, {center.y})"
                    )
                    for i, corner in enumerate(corners, start=1):
                        self.get_logger().info(
                            f"  Corner {i}: x={corner.x}, y={corner.y}"
                        )
            except AttributeError as e:
                self.get_logger().error(f"Error accessing detection fields: {e}")


def main(args=None):
    rclpy.init(args=args)
    tag_logger = TagLogger()
    rclpy.spin(tag_logger)
    tag_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import ReliabilityPolicy, QoSProfile
import csv  # Import CSV for logging
import math

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # LaserScan subscriber
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Odometry subscriber
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # AprilTag detections subscriber
        self.apriltag_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.laser_forward = 0
        self.odom_data = 0
        self.pose_saved = ''
        self.cmd = Twist()
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Logging position and tag detection data
        self.log_file = open('tag_positions.csv', 'w', newline='')  # CSV file to log data
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['Time', 'X_Position', 'Y_Position', 'Tag_ID', 'Tag_Detected'])

    # LaserScan callback
    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = [3.5 if reading == float('Inf') else (0.0 if math.isnan(reading) else reading) for reading in scan]

    # Odometry callback
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.pose_saved = position
        self.get_logger().info(f'Self position: {position.x}, {position.y}, {position.z}')

    # AprilTag detections callback (logs tag detections to CSV)
    def apriltag_callback(self, msg):
        if msg.detections:
            for detection in msg.detections:
                tag_id = detection.id
                self.log_writer.writerow([
                    self.get_clock().now().to_msg().sec, 
                    self.pose_saved.x, 
                    self.pose_saved.y, 
                    tag_id, 
                    'Yes'
                ])
                self.get_logger().info(f"Detected Tag ID: {tag_id} at position X: {self.pose_saved.x}, Y: {self.pose_saved.y}")
        else:
            self.log_writer.writerow([
                self.get_clock().now().to_msg().sec, 
                self.pose_saved.x, 
                self.pose_saved.y, 
                'None', 
                'No'
            ])
            self.get_logger().info("No tags detected")

    # Main control loop (movement logic)
    def timer_callback(self):
        if not self.scan_cleaned:
            self.turtlebot_moving = False
            return

        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3 if left_lidar_min < right_lidar_min else -0.3
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.07
            self.cmd.angular.z = -0.3 if right_lidar_min > left_lidar_min else 0.3
        else:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0

        self.publisher_.publish(self.cmd)
        self.get_logger().info(f'Publishing: {self.cmd}')

    def stop_robot(self):
        """Stop the robot if the tag is found."""
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info("Stopping robot as tag was found")
        self.destroy_node()

    def destroy_node(self):
        self.log_file.close()  # Close the CSV file when the node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    rclpy.spin(random_walk_node)
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

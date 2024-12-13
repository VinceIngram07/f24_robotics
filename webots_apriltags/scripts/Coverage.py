def apriltag_callback(self, msg):
    if msg.detections:
        for detection in msg.detections:
            tag_id = detection.id
            family = detection.family
            centre = detection.centre
            corners = detection.corners

            self.get_logger().info(f"Detected Tag Family: {family}, ID: {tag_id}")
            self.get_logger().info(f"Centre: x={centre.x}, y={centre.y}")
            self.get_logger().info("Corners:")
            for i, corner in enumerate(corners):
                self.get_logger().info(f"  Corner {i + 1}: x={corner.x}, y={corner.y}")
            
            # Log the position from odometry when the tag is detected
            x = self.pose_saved.x
            y = self.pose_saved.y
            self.get_logger().info(f"Tag detected at position: x={x}, y={y}")
            
            # Save position of detected tag to a list (optional: store in a file)
            with open("tag_positions.csv", "a") as f:
                f.write(f"{tag_id}, {x}, {y}\n")
    else:
        self.get_logger().info("No tags detected")

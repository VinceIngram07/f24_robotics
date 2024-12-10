# Random2.py Algorithm for AprilTag Detection and Random Walk

## Description
The `Random2.py` script is a ROS 2 node designed to control a TurtleBot3 robot in a simulated or physical environment. The robot performs the following tasks:
- **AprilTag Detection**: Detects AprilTags in the environment and logs the tag family, ID, center, and corner coordinates.
- **LIDAR-Based Navigation**: Uses LIDAR data to avoid obstacles and navigate randomly.
- **Odometry Logging**: Logs the robot's position for tracking.

This script is ideal for assignments requiring AprilTag detection combined with autonomous navigation.

---

## Features
1. **AprilTag Detection**:
   - Logs detected tag family, ID, center position, and corner coordinates.

2. **LIDAR-Based Movement**:
   - Avoids obstacles by turning when objects are detected within a safe distance.
   - Moves randomly when the path ahead is clear.

3. **Odometry Logging**:
   - Tracks and logs the robot's position in real-time.

---

## Algorithm Overview
1. **Initialization**:
   - Subscribes to:
     - LIDAR (`/scan`) for obstacle detection.
     - Odometry (`/odom`) for robot position tracking.
     - AprilTag detections (`/detections`) for tag logging.
   - Publishes velocity commands to the `/cmd_vel` topic.

2. **LIDAR-Based Navigation**:
   - Checks LIDAR data to determine the distance to obstacles.
   - Implements logic to move forward, turn left, or turn right based on obstacle proximity.

3. **AprilTag Detection**:
   - Listens to `/detections` and logs tag family, ID, center coordinates, and corner points.

4. **Odometry Logging**:
   - Continuously logs the robot’s position (`x`, `y`, `z`) to provide feedback on movement.

---

## How to Run the Package

### 1. **Setup Environment**
1. Clone the repository:
   ```bash
   git clone https://github.com/VinceIngram07/f24_robotics.git

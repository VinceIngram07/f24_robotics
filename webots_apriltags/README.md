# **Random2.py Algorithm for Autonomous Navigation and Coverage Analysis**

## **Description**
The `Random2.py` script is a ROS 2 node designed to control a TurtleBot3 robot in a simulated or physical environment. The robot performs the following tasks:
- **Random Walk Navigation**: Uses LIDAR data to avoid obstacles and navigate randomly through the environment.
- **Path Tracking and Coverage Analysis**: Logs the robot's position at each time step, which can later be analyzed for coverage metrics.

This script is designed to support assignments requiring exploration and path coverage analysis in various environments.

---

## **Features**
1. **Random Walk Navigation**:
   - Uses LIDAR-based obstacle detection to avoid collisions.
   - Implements wall-following behavior to ensure consistent exploration of the environment.
   - Publishes velocity commands to the `/cmd_vel` topic to control the TurtleBot3's movement.

2. **Path Tracking and Logging**:
   - Tracks the robot's position using odometry data from the `/odom` topic.
   - Logs timestamped position data (`x`, `y` coordinates) to a CSV file to enable coverage analysis.

3. **Obstacle Avoidance**:
   - Uses LIDAR sensor data from the `/scan` topic to detect obstacles in the robot's path.
   - Turns left or right to avoid obstacles when an object is detected within a safe distance.

---

## **Algorithm Overview**
1. **Initialization**:
   - Subscribes to:
     - LIDAR (`/scan`) for obstacle detection.
     - Odometry (`/odom`) for robot position tracking.
   - Publishes velocity commands to the `/cmd_vel` topic.

2. **LIDAR-Based Navigation**:
   - Checks LIDAR data to determine the distance to obstacles.
   - Implements logic to move forward, turn left, or turn right based on obstacle proximity.

3. **Path Tracking and Logging**:
   - Continuously logs the robot’s position (`x`, `y` coordinates) to a CSV file.
   - Timestamped positions allow for coverage analysis, where covered area is calculated as a percentage of free space explored.

---

## **How to Run the Package**

### **1. Setup Environment**
1. **Clone the repository**:
   ```bash
   git clone https://github.com/VinceIngram07/f24_robotics.git
   ```
2 **Navigate to the package directory**:
```bash
cd f24_robotics/webots_apriltags
```
3. **Install dependencies**:

```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build the package**:
```
colcon build --packages-select webots_apriltags
```
5. **Source the setup file**:
```
source ~/workspaces/install/setup.bash
```

### How to Launch the Simulation
**Launch Webots with the specific world file**:

```
ros2 launch webots_apriltags webots_apriltags.launch.py world:=maze.wbt
```
**Start the Random2.py controller to drive the TurtleBot3**:
```
ros2 run webots_apriltags Random2.py
```

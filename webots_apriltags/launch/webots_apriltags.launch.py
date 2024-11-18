import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # LIDAR (Hokuyo sensor or equivalent)
    lidar_node = Node(
        package='hlds_laser_publisher',
        executable='hlds_laser_publisher',
        name='hlds_laser_publisher',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}],
    )

    # Camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # AprilTag detection
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('/image_rect', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{
            'tag_family': 'tag36h11',
            'tag_size': 0.173,  # Update based on the tag's size
            'max_hamming': 0,
        }],
    )

    # Diffdrive controller (optional if manual teleop is needed)
    cmd_vel_publisher = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo/Webots) clock if true'
        ),
        robot_state_publisher,
        lidar_node,
        camera_node,
        apriltag_node,
        cmd_vel_publisher,
    ])

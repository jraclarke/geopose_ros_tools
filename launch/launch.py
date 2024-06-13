from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',  # ROS2 package for static_transform_publisher
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['--x', '-437284', '--y', '-115644', '--z' ,'0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', '/world', '--child-frame-id','/map',]
        ),
        Node(
            package='geopose_ros_tools',
            executable='publish_geopose',
            name='publish_geopose',
            output='screen'  # Assuming ROS2 logging replaces screen output
        ),
        Node(
            package='geopose_ros_tools',
            executable='geopose_to_osgb36',
            name='geopose_to_osgb36',
            output='screen'  # Assuming ROS2 logging replaces screen output
        ),
        Node(
            package='geopose_ros_tools',
            executable='publish_ply_pointcloud',
            name='publish_ply_pointcloud',
        ),
    ])

# Launch the nodes
ld = generate_launch_description()
print(ld)  # Optional: Print the launch description for debugging
#subprocess.call(["ros2", "lifecycle", "set", "publish_ply_pointcloud", "shutdown"])

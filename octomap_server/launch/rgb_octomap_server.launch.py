from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='rgb_octomap_server',
            name='rgb_octomap_server',
            output='screen',
            parameters=[
                {
                    'resolution': 0.03,  # Octree resolution in meters
                    'camera_frame': 'base_link',  # Frame of the camera
                    'world_frame': 'odom',  # World frame
                    'pointcloud_topic': '/camera/depth_registered/points'  # PointCloud2 topic
                }
            ]
        )
    ])
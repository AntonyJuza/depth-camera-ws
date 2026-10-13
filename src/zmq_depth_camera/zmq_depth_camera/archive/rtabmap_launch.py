import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info'),

        # Start RTAB-Map Node
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            remappings=[
                ('/camera/depth/image_raw', LaunchConfiguration('camera_topic')),
                ('/camera/depth/camera_info', LaunchConfiguration('camera_info_topic')),
            ],
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': False,  # Assuming depth camera only
                'RGBD/LinearUpdate': 1.0,
                'RGBD/Threshold': 0.2,
                'Mem/IncrementalMemory': True,
            }]
        ),
    ])


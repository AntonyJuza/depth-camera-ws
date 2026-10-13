from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {"frame_id": "camera_link"},
                {"subscribe_stereo": True},
                {"stereo_namespace": "/camera"},
                {"approx_sync": False},
                {"odom_topic": "/odom"},
                {"publish_tf": True},
            ],
            remappings=[
                ("/camera/left/image_rect", "/camera/ir/image_raw"),
                ("/camera/right/image_rect", "/camera/raw/image_raw"),
                ("/camera/left/camera_info", "/camera/left/camera_info"),
                ("/camera/right/camera_info", "/camera/right/camera_info"),
            ]
        )
    ])


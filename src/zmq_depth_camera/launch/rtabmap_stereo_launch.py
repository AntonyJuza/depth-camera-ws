from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='camera_link', description='Base frame ID for RTAB-Map'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic'),
        DeclareLaunchArgument('approx_sync', default_value='true', description='Approximate synchronization'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'subscribe_stereo': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'approx_sync': LaunchConfiguration('approx_sync'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_tf': True,
                'queue_size': 10,
                'Rtabmap/DetectionRate': '5.0',
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_rect'),
                ('right/image_rect', '/camera/right/image_rect'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info'),
                ('odom', LaunchConfiguration('odom_topic')),
            ]
        )
    ])

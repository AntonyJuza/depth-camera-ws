from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': False,
                'approx_sync': True,
                'queue_size': 10,
                'Mem/IncrementalMemory': True,
                'RGBD/LinearUpdate': 0.1,
                'RGBD/LinearSpeed': 0.1,
                'RGBD/AngularUpdate': 0.1,
                'RGBD/AngularSpeed': 0.1,
                'DetectionRate': 5.0,
                'RGBD/DepthEnabled': True,
                'RGBD/CameraFov': 60,
            }],
            remappings=[
                ('/rgb/image', '/camera/ir/image_raw'),          # Use IR as RGB substitute
                ('/rgb/camera_info', '/camera/left/camera_info'), # Left camera info
                ('/depth/image', '/camera/depth/image_raw'),
                ('/depth/camera_info', '/camera/depth/camera_info'),
                ('/odom', '/odom'), # Make sure you have odom or TF set correctly
            ]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen',
            parameters=[{
                'frame_id': 'map',
            }]
        )
    ])


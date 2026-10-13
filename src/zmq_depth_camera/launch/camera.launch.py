import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('zmq_depth_camera')
    default_params_file = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to ROS 2 parameters file'
        ),
        DeclareLaunchArgument(
            'zmq_url',
            default_value='tcp://10.11.11.1:20000',
            description='ZeroMQ stream URL'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='true',
            description='Enable fast internal PointCloud2 generation'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish optical frame static transforms'
        ),
        DeclareLaunchArgument(
            'publish_base_tf',
            default_value='false',
            description='Publish static transform from base_link to camera_link'
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='Camera base frame ID'
        ),

        # ZMQ Depth Camera Node
        Node(
            package='zmq_depth_camera',
            executable='camera_node',
            name='zmq_depth_camera_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'zmq_url': LaunchConfiguration('zmq_url'),
                    'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'publish_base_tf': LaunchConfiguration('publish_base_tf'),
                    'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                }
            ],
            emulate_tty=True
        ),
    ])

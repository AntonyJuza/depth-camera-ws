import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('zmq_depth_camera')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'camera_view.rviz')
    camera_launch_file = os.path.join(pkg_share, 'launch', 'camera.launch.py')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz2 configuration file'
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

        # Include camera.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file),
            launch_arguments={
                'zmq_url': LaunchConfiguration('zmq_url'),
                'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
            }.items()
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            emulate_tty=True
        ),
    ])

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'zmq_depth_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pyzmq',
    ],
    zip_safe=True,
    maintainer='juza',
    maintainer_email='juza@todo.todo',
    description='ZMQ camera publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_pub = zmq_depth_camera.ir_pub:main',
            'ir_pub_working = zmq_depth_camera.ir_pub_working_version:main',
            'camera_info = zmq_depth_camera.stereo_camera_info_pub:main',
            'depth_node = zmq_depth_camera.depth_ir:main',
        ],
    },
)


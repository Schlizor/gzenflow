from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gzenflow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')), 
        ('share/' + package_name + '/launch', ['launch/gzenflow_launch.py']),
    ],
    install_requires=['setuptools', 'eclipse-zenoh', 'pyyaml', 'json5', 'rosidl-interface-pkgs'],
    zip_safe=True,
    maintainer='Thomas S.',
    maintainer_email='Schlichting25@gmail.com',
    description='Dynamic GStreamer and Zenoh stream control in ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
         'console_scripts': [
                'controller = gzenflow.controller:main',            
                'network_manager = gzenflow.network_manager:main',
                'ros2_gstreamer_streamer = gzenflow.ros2_gstreamer_streamer:main',
        ],
    },
)

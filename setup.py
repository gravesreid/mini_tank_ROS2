from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_tank_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('mini_tank_ros/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reid',
    maintainer_email='rgraves@andrew.cmu.edu',
    description='mini tank driver for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tank_driver = mini_tank_ros2.tank_driver:main',
            'video_subscriber = mini_tank_ros2.video_subscriber:main',
            'joystick_driver = mini_tank_ros2.joystick_driver:main',
            'object_detection_driver = mini_tank_ros2.object_detection_driver:main',
            'object_detection_subscriber = mini_tank_ros2.object_detection_subscriber:main',
            'image_save_subscriber = mini_tank_ros2.image_save_subscriber:main',
            'robot_object_detection_subscriber = mini_tank_ros2.robot_object_detection_subscriber:main',
            'magnetometer_node = mini_tank_ros2.magnetometer_node:main',
            'mag_controller_node = mini_tank_ros2.mag_controller_node:main',
        ],
    },
)


from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_tank_ROS2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('mini_tank_ROS2/*.py')),
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
            'tank_driver = mini_tank_ROS2.tank_driver:main',
            'video_subscriber = mini_tank_ROS2.video_subscriber:main',
            'joystick_driver = mini_tank_ROS2.joystick_driver:main',
        ],
    },
)


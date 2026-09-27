import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'smartphone_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Non-Python files are NOT installed automatically. Without these two lines
        # `ros2 launch smartphone_robot velocity_demo.launch.py` fails with "file
        # not found", because ros2 launch looks in install/share/<pkg>/, never in
        # src/. Every new launch file or config file is picked up by these globs.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdurahmon',
    maintainer_email='cognilabsco@gmail.com',
    description='Smartphone sensors + OpenCV + Gazebo + SLAM + Nav2 on ROS 2 Jazzy.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # Each line maps  <executable name> = <python module>:<function>
        # The left-hand name is the second word of `ros2 run smartphone_robot <name>`.
        'console_scripts': [
            'velocity_publisher = smartphone_robot.velocity_publisher:main',
            'velocity_monitor = smartphone_robot.velocity_monitor:main',
            'drive_distance_server = smartphone_robot.drive_distance_server:main',
            'wander_node = smartphone_robot.wander_node:main',
        ],
    },
)

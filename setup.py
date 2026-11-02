from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'stretch4_rtabmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json') + glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hello-robot',
    maintainer_email='support@hello-robot.com',
    description='RTAB-Map localization and mapping on GLIM odometry for Nav2 on Stretch',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_to_base_frame = stretch4_rtabmap.sensors.imu_to_base_frame:main',
            'robot_self_mask = stretch4_rtabmap.sensors.robot_self_mask:main',
            'rtabmap_loop_monitor = stretch4_rtabmap.localization.rtabmap_loop_monitor:main',
            'mapped_navigator = stretch4_rtabmap.navigation.mapped_navigator:main',
            'rtabmap_supervisor = stretch4_rtabmap.localization.rtabmap_supervisor:main',
            'memory_monitor = stretch4_rtabmap.diagnostics.memory_monitor:main',
            'export_map = stretch4_rtabmap.tools.export_map:main',
            'jetson = stretch4_rtabmap.tools.jetson:main',
        ],
    },
)

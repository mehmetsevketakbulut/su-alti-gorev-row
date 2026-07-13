from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rov_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sevket',
    maintainer_email='sevket@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follower = rov_vision.line_follower:main',
            'video_publisher = rov_vision.video_publisher:main',
            'autonomous_driver = rov_vision.autonomous_driver:main',
            'mission_orbit = rov_vision.mission_orbit:main',
            'video_mission = rov_vision.video_mission:main',

            'distance_publisher = rov_vision.distance_publisher:main',
            'test_serial = rov_vision.test_serial_jetson:main',
        ],
    },
)

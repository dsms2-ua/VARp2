from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vfh_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'waypoints'),
            glob('waypoints/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raúl Boix',
    maintainer_email='pepetrola2015@gmail.com',
    description='VFH+ navigation for TurtleBot3 race circuit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_recorder = vfh_navigation.waypoint_recorder:main',
            'vfh_node          = vfh_navigation.vfh_node:main',
        ],
    },
)

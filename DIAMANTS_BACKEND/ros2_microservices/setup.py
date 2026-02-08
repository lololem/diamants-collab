from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'diamants_microservices'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DIAMANTS Team',
    maintainer_email='diamants@project.dev',
    description='DIAMANTS ROS2 Microservices - Collaborative SLAM Multi-Agent System',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_controller = diamants_microservices.swarm_controller:main',
            'position_broadcaster = diamants_microservices.position_broadcaster:main',
            'slam_fusion = diamants_microservices.slam_fusion:main',
            'mission_coordinator = diamants_microservices.mission_coordinator:main',
        ],
    },
)

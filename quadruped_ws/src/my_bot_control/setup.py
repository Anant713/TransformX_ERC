from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_bot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ROS 2 to find the package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='IK solver and control nodes for my_bot quadruped',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registers the node so it can be launched with:
            #   ros2 run my_bot_control ik_solver_node
            'ik_solver_node = my_bot_control.ik_solver_node:main',
        ],
    },
)

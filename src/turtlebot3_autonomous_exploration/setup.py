from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_autonomous_exploration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anish Kumar',
    maintainer_email='anish@example.com',
    description='Autonomous exploration package for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_explorer = turtlebot3_autonomous_exploration.autonomous_explorer:main',
            'exploration_controller = turtlebot3_autonomous_exploration.exploration_controller:main',
            'exploration_recovery = turtlebot3_autonomous_exploration.exploration_recovery:main',
            'exploration_supervisor = turtlebot3_autonomous_exploration.exploration_supervisor:main',
            'frontier_detector = turtlebot3_autonomous_exploration.frontier_detector:main',

        ],
    },
)
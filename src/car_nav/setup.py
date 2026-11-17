from glob import glob
import os

from setuptools import setup

package_name = 'car_nav'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name, package_name + '.gmr_lib'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='car_nav maintainers',
    maintainer_email='maintainers@example.com',
    description='GMR-RRT*/A* planners and TurtleBot3 demo helpers (SLAM, waypoints)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = car_nav.astar_planner:main',
            'goal_client = car_nav.goal_client:main',
            'initial_pose_publisher = car_nav.initial_pose_publisher:main',
            'auto_explore = car_nav.auto_explore:main',
            'frontier_explore = car_nav.frontier_explore:main',
            'mission_runner = car_nav.mission_runner:main',
            'gmr_planner_node = car_nav.gmr_planner_node:main',
            'language_mission = car_nav.language_mission_node:main',
            'language_gui = car_nav.language_gui:main',
        ],
    },
)

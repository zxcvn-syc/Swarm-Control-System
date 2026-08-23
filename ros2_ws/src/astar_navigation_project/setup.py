from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astar_navigation_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsako',
    maintainer_email='hsako@todo.com',
    description='A* and D* Lite navigation with UAV/UGV support',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = astar_navigation_project.planner_node:main',
            'dstar_benchmark = astar_navigation_project.dstar_benchmark_node:main',
        ],
    },
)

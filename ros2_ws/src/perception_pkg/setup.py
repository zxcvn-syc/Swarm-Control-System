from setuptools import find_packages, setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracker_node.launch.py']),
        ('share/' + package_name + '/config', ['config/tracker_node.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Swarm Control System Team',
    maintainer_email='swarm@example.com',
    description=(
        'Perception pipeline nodes: YOLOv8 detector and DeepSORT / '
        'BoT-SORT tracker publishing swarm_interfaces/TargetTrackArray.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tracker_node = perception_pkg.tracker_node:main',
        ],
    },
)
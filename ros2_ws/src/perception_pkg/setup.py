from glob import glob
from pathlib import Path

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py

package_name = 'perception_pkg'


class _BuildPy(_build_py):
    """Install cvtrack presets beside its vendored Python package."""

    def run(self):
        super().run()
        source_dir = Path(__file__).parent / 'cvtrack' / 'configs'
        target_dir = Path(self.build_lib) / 'cvtrack' / 'configs'
        self.mkpath(str(target_dir))
        for source_path in source_dir.glob('*.yaml'):
            self.copy_file(str(source_path), str(target_dir / source_path.name))

setup(
    name=package_name,
    version='0.1.0',
    packages=(
        find_packages(exclude=['test', 'tests'])
        + find_packages(where='cvtrack/src')
    ),
    package_dir={'cvtrack': 'cvtrack/src/cvtrack'},
    package_data={'cvtrack': ['configs/*.yaml']},
    cmdclass={'build_py': _BuildPy},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
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
            'coord_transform_node = perception_pkg.coord_transform_node:main',
        ],
    },
)

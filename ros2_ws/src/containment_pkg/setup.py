from setuptools import setup


package_name = 'containment_pkg'


setup(
    name=package_name,
    version='0.0.0',

    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/containment.launch.py']
        ),
        (
            'share/' + package_name + '/config',
            ['config/containment.yaml']
        ),
    ],

    zip_safe=True,

    maintainer='chen',

    description='Static Voronoi UAV containment demo',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'enclosure_node = containment_pkg.enclosure_node:main',
            'mock_platform_pub = containment_pkg.mock_platform_pub:main',
        ],
    },
)

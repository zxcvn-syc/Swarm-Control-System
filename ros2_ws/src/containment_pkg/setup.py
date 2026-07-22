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
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='chen',

    description='Static Voronoi UAV containment demo',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'static_voronoi_uav = containment_pkg.static_voronoi_uav:main',
        ],
    },
)

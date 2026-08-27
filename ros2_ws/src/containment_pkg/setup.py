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
            [
                'launch/containment.launch.py',
                'launch/ugv_block_demo.launch.py',
                'launch/full_loop_demo.launch.py',
                'launch/rfly_enclosure.launch.py',
                'launch/escape_eval.launch.py',
                'launch/escape_eval_sitl.launch.py',
            ]
        ),
        (
            'share/' + package_name + '/config',
            ['config/containment.yaml', 'config/three_scene_config.yaml']
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
            'target_pub = containment_pkg.target_pub:main',
            'enclosure_command_bridge = containment_pkg.enclosure_command_bridge:main',
            'platform_state_merger = containment_pkg.platform_state_merger:main',
            'escape_test_node = containment_pkg.escape_test_node:main',
            'containment_evaluator = containment_pkg.containment_evaluator:main',
        ],
    },
)

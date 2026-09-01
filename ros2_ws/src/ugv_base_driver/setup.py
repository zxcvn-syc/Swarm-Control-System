from setuptools import find_packages, setup

package_name = "ugv_base_driver"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/config", ["config/ugv_base_driver.yaml"]),
    (
        "share/" + package_name + "/launch",
        [
            "launch/ugv_base_driver.launch.py",
            "launch/ugv_vehicle.launch.py",
        ],
    ),
]

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=data_files,
    zip_safe=True,
    maintainer="Swarm Control System Team",
    maintainer_email="swarm@example.com",
    description=(
        "Guarded differential-drive UGV control with path following, "
        "obstacle sensing, odometry bridging, and serial base output."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ugv_base_driver = ugv_base_driver.base_driver_node:main",
            "ugv_path_follower = ugv_base_driver.path_follower_node:main",
            "ugv_obstacle_guard = ugv_base_driver.obstacle_guard_node:main",
            "ugv_odom_state_bridge = ugv_base_driver.odom_state_bridge_node:main",
        ],
    },
)

import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = "ugv_base_driver"
package_root = Path(__file__).resolve().parent

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/config", ["config/ugv_base_driver.yaml"]),
    ("share/" + package_name + "/launch", ["launch/ugv_base_driver.launch.py",
                                            "launch/ugv_path_follower.launch.py"]),
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=data_files,
    zip_safe=True,
    maintainer="Swarm Control System Team",
    maintainer_email="swarm@example.com",
    description=(
        "Differential-drive UGV base driver: /cmd_vel -> wheel speeds -> "
        "serial output, with watchdog safety and parameterized protocol."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ugv_base_driver = ugv_base_driver.base_driver_node:main",
            "ugv_path_follower = ugv_base_driver.path_follower_node:main",
        ],
    },
)

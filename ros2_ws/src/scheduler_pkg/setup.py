from setuptools import find_packages, setup

package_name = "scheduler_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/scheduler.yaml"]),
        ("share/" + package_name + "/launch", ["launch/scheduler.launch.py"]),
    ],
    zip_safe=True,
    maintainer="Swarm Control System Team",
    maintainer_email="swarm@example.com",
    description=(
        "Scheduler package: assigns perception targets to UAVs "
        "(greedy / Hungarian) and publishes swarm_interfaces/TaskAssignment."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scheduler_node = scheduler_pkg.scheduler_node:main",
            "reallocation_collector = scheduler_pkg.reallocation_collector:main",
        ],
    },
)

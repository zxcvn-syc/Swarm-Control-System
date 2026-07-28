from setuptools import setup

package_name = "planner_stub"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/planner_stub.yaml"]),
    ],
    zip_safe=True,
    maintainer="Swarm Control System Team",
    maintainer_email="swarm@example.com",
    description=(
        "Integration shim for the empty planning_pkg slot. "
        "Publishes synthetic DroneStateArray + per-drone DroneState so the "
        "three-link integration test can run end-to-end before the real "
        "planner_node lands."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "planner_stub_node = planner_stub.planner_stub_node:main",
        ],
    },
)

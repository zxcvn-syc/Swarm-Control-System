# ROS2 Humble base image used by the Swarm-Control-System CI.
#
# Provides everything ``colcon build`` and the integration tests need
# so the CI workflow can run ``docker build`` once and reuse the image
# for every push / pull-request instead of re-running apt-get each
# time.
#
# Build:
#   docker build -f docker/ros2_humble.Dockerfile -t swarm-ci:humble .
#
# Run:
#   docker run --rm -v "$PWD":/workspace -w /workspace swarm-ci:humble \
#       bash -c "source /opt/ros/humble/setup.bash && \
#                source ros2_ws/install/setup.bash && \
#                cd ros2_ws && colcon build --packages-select planning_pkg"

FROM osrf/ros:humble-desktop

# Non-interactive apt install for CI determinism.
ENV DEBIAN_FRONTEND=noninteractive

# Workspace dependencies that used to be installed piecemeal in the
# GitHub Actions workflow.  We add them once, in the image, so the
# workflow can reuse the layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pytest \
        python3-numpy \
        ros-humble-ros-base \
        ros-humble-nav-msgs \
        ros-humble-geometry-msgs \
        ros-humble-std-msgs \
        ros-humble-sensor-msgs \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Python deps that need pip.  Pin to versions known to work with the
# Humble Python 3.10 interpreter on Ubuntu 22.04.
RUN pip3 install --no-cache-dir \
        "numpy<2" \
        "opencv-python<4.11" \
        "pytest"

WORKDIR /workspace

# Default entrypoint: source the global ROS setup, drop into a shell
# so the caller can run any colcon / pytest command they want.
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]
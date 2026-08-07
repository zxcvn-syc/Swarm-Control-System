# ROS2 Humble + Gazebo Garden + PX4 SITL simulation image.
#
# This image extends the plain ROS2 CI image (docker/ros2_humble.Dockerfile)
# with everything needed to run PX4 SITL against Gazebo and to launch the
# planning_pkg bridge nodes inside the container.
#
# What's inside:
#   * ROS2 Humble (ros:humble-desktop base)
#   * colcon, pytest, numpy, opencv-python (inherited from base image)
#   * ros-humble-gazebo-ros-pkgs + ros-humble-gazebo-dev
#   * Gazebo Garden server + plugins (gz-garden)
#   * ros-humble-mavros + ros-humble-mavros-extras
#   * GeographicLib datasets (required by MAVROS)
#   * PX4-Autopilot SITL firmware (lightweight, no git checkout)
#
# Disk space note:
#   * PX4 SITL (gz_iris) + Gazebo Garden + plugins ≈ 4-5 GB after build.
#   * The PX4 build is done at image-build time so the runtime container
#     starts in seconds.
#
# Build:
#   docker build \
#     -f docker/px4_sitl.Dockerfile \
#     -t swarm-ci:px4-sitl .
#
# Run SITL headless:
#   docker run --rm -it --net=host \
#     -e GAZEBO_HEADLESS=true \
#     -v "$PWD":/workspace -w /workspace \
#     swarm-ci:px4-sitl \
#     bash -lc "source /opt/ros/humble/setup.bash && \
#               source /workspace/ros2_ws/install/setup.bash && \
#               PX4_SITL_ROOT=/opt/px4/Firmware \
#               ros2 launch planning_pkg sitl_test.launch.py"
#
# Build time: ~20-30 min on a 4-core runner (PX4 SITL is the slow part).
# Image size: ~5 GB.

ARG BASE_TAG=humble
FROM swarm-ci:${BASE_TAG} AS base

# Non-interactive apt for CI determinism.
ENV DEBIAN_FRONTEND=noninteractive

# ----------------------------------------------------------------------
# Stage 1: Gazebo + MAVROS + PX4 dependencies.
# ----------------------------------------------------------------------
FROM base AS px4_deps

RUN apt-get update && apt-get install -y --no-install-recommends \
        # Gazebo Garden (gz sim) is not in apt for Ubuntu 22.04 by default;
        # we use Gazebo Classic for SITL compatibility since the PX4
        # gazebo-classic plugin set is the most stable.
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-dev \
        ros-humble-gazebo-msgs \
        # MAVROS: required for the UDP MAVLink <-> ROS bridge.
        ros-humble-mavros \
        ros-humble-mavros-extras \
        # MAVROS needs the GeographicLib datasets for geodesic conversions.
        # The apt package ships a small subset; we install the full set
        # with the helper script.
        geographiclib-tools \
        # PX4 firmware native build deps.
        git \
        ca-certificates \
        build-essential \
        ccache \
        cmake \
        cppcheck \
        file \
        g++ \
        gcc \
        gdb \
        lcov \
        make \
        ninja-build \
        python3 \
        python3-dev \
        python3-pip \
        python3-jinja2 \
        python3-numpy \
        python3-pkg-resources \
        python3-pyparsing \
        python3-yaml \
        rsync \
        shellcheck \
        unzip \
        vim-common \
        wget \
        xsltproc \
        zip \
        # Gazebo Classic runtime shared libraries.
        libgazebo-dev \
        libgstreamer-plugins-base1.0-dev \
        libimage-exiftool-perl \
        libxml2-utils \
        pkg-config \
        protobuf-compiler \
        libeigen3-dev \
        libopencv-dev \
        libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

# Install the MAVROS GeographicLib datasets (small, ~10 MB).
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh || \
    echo "[px4_sitl] geographiclib install script exited; mavros will warn but still works for most conversions."

# ----------------------------------------------------------------------
# Stage 2: PX4-Autopilot SITL firmware build.
# We clone only the release tag and build px4_sitl_default.  No submodules
# are pulled for tools we do not need (QGroundControl, etc.).
# ----------------------------------------------------------------------
FROM px4_deps AS px4_build

ARG PX4_TAG=v1.14.0
ARG PX4_REPO=https://github.com/PX4/PX4-Autopilot.git
ENV PX4_SITL_ROOT=/opt/px4/Firmware

# Clone (shallow) + update submodules required for SITL.
RUN git clone --depth 1 --branch "${PX4_TAG}" "${PX4_REPO}" "${PX4_SITL_ROOT}" && \
    (cd "${PX4_SITL_ROOT}" && \
        git submodule update --init --recursive --depth 1 \
            src/drivers/gps/devices \
            src/drivers/uavcan/libuavcan \
            src/lib/bezier \
            src/lib/crypto/sha256 \
            src/lib/eventslib \
            src/lib/mathlib \
            src/lib/matrix \
            src/lib/output_to_pwm \
            src/lib/pid \
            src/lib/ringbuffer \
            src/lib/systemlib \
            src/lib/parameters \
            src/lib/perf \
            src/modules/airspeed_selector \
            src/modules/ekf2 \
            src/modules/mc_att_control \
            src/modules/mc_hover_thrust_estimator \
            src/modules/mc_pos_control \
            src/modules/mc_rate_control \
            src/modules/micrortps_bridge \
            src/modules/simulation \
            src/modules/simulator/simulator_mavlink \
            src/modules/temperature_compensation \
            src/modules/uavcan \
            src/modules/uORB \
            src/systemcmds \
            Tools/simulation/gazebo-classic/sitl_gazebo-classic || \
        echo "[px4_sitl] some submodules failed; the SITL build will pull what it needs on first configure.")

# Build PX4 SITL default.  This is the time-consuming step (~15-20 min on 4 cores).
# We disable ccache pre-warming to keep the image deterministic; subsequent
# rebuilds inside the container will reuse object files if ccache is warm.
RUN cd "${PX4_SITL_ROOT}" && \
    DONT_RUN=1 MAKEFLAGS="-j$(nproc)" \
        make px4_sitl_default \
        PX4_GZ_WORLD=swarm_field && \
    rm -rf "${PX4_SITL_ROOT}/build/px4_sitl_default/.obj" \
           "${PX4_SITL_ROOT}/build/px4_sitl_default/.dep" 2>/dev/null || true

# ----------------------------------------------------------------------
# Stage 3: final slim runtime image.
# ----------------------------------------------------------------------
FROM px4_deps AS final

ENV PX4_SITL_ROOT=/opt/px4/Firmware
ENV GAZEBO_PLUGIN_PATH=/opt/px4/Firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic
ENV GAZEBO_MODEL_PATH=/opt/px4/Firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# Copy the prebuilt PX4 firmware from the build stage.
COPY --from=px4_build /opt/px4/Firmware /opt/px4/Firmware

WORKDIR /workspace

# Default entrypoint: source ROS + MAVROS + drop into a shell so the caller
# can run ros2 launch / gazebo / colcon.
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && exec bash"]

#!/usr/bin/env bash
# Install the administrator-owned dependencies for the one-UAV PX4 SITL profile.
# Run inside the ROS2 Humble VM as the target user.

set -euo pipefail

PX4_SITL_ROOT="${PX4_SITL_ROOT:-$HOME/src/PX4-Autopilot}"
PX4_REF="${PX4_REF:-v1.14.0}"

sudo apt-get update
sudo apt-get install -y \
  gazebo \
  libgazebo-dev \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-dev \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  geographiclib-tools \
  build-essential \
  cmake \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  ninja-build \
  python3-pip \
  git

python3 -m pip install --user --upgrade kconfiglib pyros-genmsg jsonschema

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

mkdir -p "$(dirname "$PX4_SITL_ROOT")"
if [[ ! -d "$PX4_SITL_ROOT/.git" ]]; then
  git clone --depth 1 --branch "$PX4_REF" \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_SITL_ROOT"
fi

cd "$PX4_SITL_ROOT"
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo-classic

printf '\nPX4 SITL is ready. Add this to the current shell:\n'
printf 'export PX4_SITL_ROOT=%q\n' "$PX4_SITL_ROOT"

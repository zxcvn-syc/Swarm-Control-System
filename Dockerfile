# Top-level Dockerfile for Swarm-Control-System simulation environment.
#
# This file is intentionally thin: it delegates to the canonical image
# definitions under ./docker/.  We provide it so that
#   docker build -t swarm-sim .
# works out of the box for newcomers.
#
# Available images (defined in ./docker/):
#   ros2_humble.Dockerfile  — minimal ROS2 Humble for `colcon build` + tests
#   px4_sitl.Dockerfile     — full Gazebo + PX4 SITL + MAVROS stack (≈5 GB)
#
# Build the SITL image by default::

#   docker build -f Dockerfile -t swarm-sim:sitl .

ARG VARIANT=sitl
FROM docker/${VARIANT}.Dockerfile

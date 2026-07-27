# perception_pkg/ — workspace root directory
# This file makes perception_pkg/ a regular Python package, not a namespace package.
# Without it, Python treats the directory as a namespace package and searches
# sys.path for sub-packages, causing import conflicts with the cvtrack vendored
# packages.  The actual ROS2 package lives in perception_pkg/perception_pkg/.

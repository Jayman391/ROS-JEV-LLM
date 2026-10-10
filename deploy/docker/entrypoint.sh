#!/bin/bash
# Source ROS and the workspace, then run the command (compose/k8s pass one node per container).
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ws/install/setup.bash
exec "$@"

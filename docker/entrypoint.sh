#!/usr/bin/env bash
# Sources the ROS 2 underlay (and the workspace overlay, once it is built)
# before handing control to whatever command was asked for.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

exec "$@"

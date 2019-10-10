#!/bin/bash
set -e
# setup ros environment
source "$ROS2_WS/install/local_setup.bash"
source "$DEPENDENCIES_WS/install/local_setup.bash"
source "$APP_WS/install/local_setup.bash"
exec "$@"

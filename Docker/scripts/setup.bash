#!/bin/bash
echo "---<setup.bash>---"
set -e

rm -r -f build install log | sed 's/^/  /'

source /opt/ros/foxy/install/setup.bash | sed 's/^/  /'

sudo apt-get update | sed 's/^/  /'
rosdep update | sed 's/^/  /'
rosdep install --from-paths src --ignore-src -y | sed 's/^/  /'

source /opt/ros/foxy/install/setup.bash | sed 's/^/  /'
echo "---</setup.bash>---"
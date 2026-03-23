#!/bin/bash
##############################################################################
# 
# v1.0 [William Arias] - initial version
#
##############################################################################

source ../arInterfaces/install/setup.bash

rm -rf ./log
rm -rf ./install
rm -rf ./build

colcon build --packages-select gamepad4ros2 \
  --base-paths ./ \
  --log-base ./log \
  --build-base ./build \
  --install-base ./install

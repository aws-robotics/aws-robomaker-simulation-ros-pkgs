#! /usr/bin/env bash

# the following variable will only be set by AWS RoboMaker, when run in AWS RoboMaker
# AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE

if [[ -n ${AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE} ]]; then
    BUNDLE_CURRENT_PREFIX=${AWS_ROBOMAKER_WORLDFORGE_SETUP_OVERRIDE:-/opt/robomaker/worldforge/$ROS_DISTRO}
    source ${BUNDLE_CURRENT_PREFIX}/setup.sh
    unset BUNDLE_CURRENT_PREFIX
fi

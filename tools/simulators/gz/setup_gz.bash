#!/bin/bash
#
# Setup environment to make Crazyflie visible to Gazebo.
# Derived from PX4 setup_gazebo.sh

if [ "$#" != 2 ]; then
    echo -e "usage: source setup_gz.bash src_dir build_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2

# setup gz environment and update package path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:${BUILD_DIR}/build_crazyflie_sitl_gz
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${SRC_DIR}/tools/simulators/gz/crazyflie_sitl_gz/models:${SRC_DIR}/tools/simulators/gz/crazyflie_sitl_gz/worlds
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_crazyflie_sitl_gz
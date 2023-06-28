#!/bin/bash

IMAGE_NAME="duckietown/dt-gui-tools:daffy-amd64"
RUN_PATH="$(dirname -- "${BASH_SOURCE[0]}")"

RED='\033[0;32m'
NC='\033[0m' # No Color

if [ -n "$1" ]
then 
    echo -e "\n*** Run container on host and connect to duckiebot name: ${RED}${1}${NC} ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh" ${1}

    # Start container
    docker run -it --rm \
        -v ${PWD}/assets/rqt:/code/catkin_ws/src/dt-gui-tools/rqt \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
        -e DISPLAY=unix$DISPLAY \
        --network host \
        --privileged \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        -e ROS_IP=${ROS_IP} \
        -e VEHICLE_NAME=${1} \
        ${IMAGE_NAME} \
        /bin/bash
else
    echo -e "\n*** Run container on host ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh"

    # Start container
    docker run -it --rm \
        -v ${PWD}:/code/catkin_ws/src/SpinakerV1 \
        --network host \
        ${IMAGE_NAME} \
        /bin/bash
fi
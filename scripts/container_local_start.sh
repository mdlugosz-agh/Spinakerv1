#!/bin/bash

IMAGE_NAME="duckietown/spinakerv1:v2-amd64"

if [ -n "$1" ]
then 
    echo "Run container on host and connect to duckiebot name: ${1}"

    # Set enviromental varaibles
    source set_env.sh ${1}

    # Start container
    docker run --runtime=nvidia -it --rm \
        -v ${PWD}:/code/catkin_ws/src/SpinakerV1 \
        -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
        --network host \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        -e ROS_IP=${ROS_IP} \
        -e VEHICLE_NAME=${1} \
        ${IMAGE_NAME} \
        /bin/bash
else
    echo "Run container on host"

    # Set enviromental varaibles
    source set_env.sh

    # Start container
    docker run -it --rm \
        -v ${PWD}:/code/catkin_ws/src/SpinakerV1 \
        --network host \
        ${IMAGE_NAME} \
        /bin/bash
fi
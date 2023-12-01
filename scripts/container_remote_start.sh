#!/bin/bash

echo "Have to be corrected by property mount remote volume, or copy all files to image like (assets/)"

exit 1

IMAGE_NAME="duckietown/spinakerv1:latest-amd64"
RUN_PATH="$(dirname -- "${BASH_SOURCE[0]}")"

RED='\033[0;32m'
NC='\033[0m' # No Color

if [ -z "$1" ]
then 
    echo -e "\n*** container_remote_start DUCKIEBOT_NAME USER HOST ***\n"
    exit 1
fi

if [ -z "$2" ]
then 
    echo -e "\n*** container_remote_start DUCKIEBOT_NAME USER HOST ***\n"
    exit 1
fi

if [ -z "$3" ]
then 
    echo -e "\n*** container_remote_start DUCKIEBOT_NAME USER HOST ***\n"
    exit 1
fi

echo -e "\n*** Run container on host and connect to duckiebot name: ${RED}${1}${NC} on ${2}@${3}***\n"

# Set enviromental varaibles
source "${RUN_PATH}/set_env.sh" ${1}
# Start container
docker -H ssh://${2}@${3} run -it --rm \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    --network host \
    --privileged \
    -e ROS_MASTER_URI=${ROS_MASTER_URI} \
    -e ROS_IP=${ROS_IP} \
    -e VEHICLE_NAME=${1} \
    ${IMAGE_NAME} \
    /bin/bash

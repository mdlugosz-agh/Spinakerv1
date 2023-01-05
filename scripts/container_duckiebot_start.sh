#!/bin/bash

IMAGE_NAME="duckietown/spinakerv1:v2-arm64v8"

if [ -v "$1" ]
then 
    echo "Usage container_duckie_start.sh <duckiebot name>"
    exit 1
fi

echo "Run remote container on duckiebot name: ${1}"

# Set enviromental varaibles
source scripts/set_env.sh ${1}

# Start container
docker -H "${1}.local" \
    run -it --rm \
    --network host \
    -e VEHICLE_NAME=${1} \
    ${IMAGE_NAME} \
    /bin/bash
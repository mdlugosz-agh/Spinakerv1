#!/bin/bash

echo "Usage: rosbag_data [duckiebot name] [postfix] [duration=30]"

if [ -z "$1" ]
then
    echo "Set duckiebot name"
    exit 1
else
    VEHICLE_NAME=${1}
fi

if [ -z "$2" ]
then
    echo "Set postfix"
    exit 1
else 
    POSTFIX=${2}
fi

if [ -z "$3" ]
then
    DURATION=30
else 
    DURATION=${3}
fi

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
DATE="$(date -I)"

touch "${SCRIPTPATH}/../rosbag/${DATE}-${VEHICLE_NAME}-${POSTFIX}.txt"

rosbag record --duration=${DURATION} \
    -O "${SCRIPTPATH}/../rosbag/${DATE}-${VEHICLE_NAME}-${POSTFIX}.bag" \
    /${VEHICLE_NAME}/data_sync_node/out/image/compressed \
    /${VEHICLE_NAME}/data_sync_node/out/car_cmd
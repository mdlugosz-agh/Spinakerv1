#!/bin/bash
# -u $(id -u):$(id -g)

if [ -n "$1" ]
then
    PORT=${1}
else
    PORT=8888
fi

echo "Run tensorflow-gpu-jupyter container on port: ${PORT}"

docker run --runtime=nvidia -it --rm \
    -v ${PWD}:/tf/SpinakerV1 \
    -v ${PWD}/../SpinakerV1Data:/tf/SpinakerV1Data \
    -p 8888:8888 \
    spinaker_tensorflow:gpu-jupyter
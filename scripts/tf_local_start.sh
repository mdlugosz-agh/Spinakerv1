#!/bin/bash
# -u $(id -u):$(id -g)

docker run --runtime=nvidia -it --rm \
    -v ${PWD}:/code/SpinakerV1 \
    -v ${PWD}/../SpinakerV1Data:/code/SpinakerV1Data \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=unix$DISPLAY \
    --privileged \
    --network host \
    spinaker_tensorflow:gpu \
    /bin/bash
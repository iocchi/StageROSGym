#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=rosstagegym

VERSION=latest
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

echo "Running image $IMAGENAME:$VERSION ..."

docker run -it \
    --name rosstagegym --rm \
    --privileged \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v `pwd`:/home/robot/StageROSGym \
    $IMAGENAME:$VERSION


#!/bin/bash

NVIDIA_STR=""

NVIDIADIR=`ls -d1 /usr/lib/nvidia-* 2> /dev/null | head -n 1`

if [ $NVIDIADIR ]; then
  echo "Found nvidia libraries: $NVIDIADIR"
  NVIDIA_STR="-v $NVIDIADIR:$NVIDIADIR \
           --device /dev/dri"
fi

VERSION=latest
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

docker stop stage_environments &> /dev/null
docker container rm stage_environments &> /dev/null

docker create -it \
    --name stage_environments \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    $NVIDIA_STR \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    iocchi/stage_environments:$VERSION


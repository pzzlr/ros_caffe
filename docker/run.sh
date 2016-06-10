#!/bin/sh

xhost +local:root

nvidia-docker run \
  -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --publish 8080:8080 \
  --publish 8085:8085 \
  --publish 9090:9090 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/${USER}/ros_caffe:/root/catkin_ws/src/ros_caffe:ro" \
  --device /dev/video0:/dev/video0 \
  ros:indigo-ros-caffe bash

xhost -local:root

# For openni
# --device /dev/bus/usb:/dev/bus/usb \
# --privileged \

# For webcam
# --device /dev/video0:/dev/video0 \

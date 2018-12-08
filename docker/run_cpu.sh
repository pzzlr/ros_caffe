#!/bin/sh

# xhost +local:root

docker run \
  -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --publish 8080:8080 \
  --publish 8085:8085 \
  --publish 9090:9090 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/${PWD}/../ros_caffe/data:/root/catkin_ws/src/ros_caffe/ros_caffe/data" \
  --device /dev/video0:/dev/video0 \
  pzzlr/ros_caffe:cpu roslaunch ros_caffe_web ros_caffe_web.launch

# xhost -local:root

# For openni (don't forget to install openni packages or add them to dockerfile)
# --device /dev/bus/usb:/dev/bus/usb \
# --privileged \

# For webcam
# --device /dev/video0:/dev/video0 \

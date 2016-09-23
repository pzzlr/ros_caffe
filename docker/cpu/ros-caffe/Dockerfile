FROM cpu:indigo-caffe

# setup environment
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# setup keys
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
ENV ROS_DISTRO indigo
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-indigo-ros-core \
    ros-indigo-openni2-launch \
    ros-indigo-openni2-camera \
    ros-indigo-usb-cam \
    ros-indigo-rosbridge-server \
    ros-indigo-roswww \
    ros-indigo-mjpeg-server \
    python-twisted \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# clone ros-caffe project
RUN mkdir -p catkin_ws/src \
  && cd catkin_ws/src \
  && git clone https://github.com/ruffsl/ros_caffe.git

# build ros-caffe ros wrapper
ADD build.sh catkin_ws/src/ros_caffe/build.sh
RUN ./catkin_ws/src/ros_caffe/build.sh

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

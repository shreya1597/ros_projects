FROM ros:melodic-robot
ENV ROS_DISTRO melodic

ENV DEBIAN_FRONTEND noninteractive

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN set -e

RUN apt-get update && apt-get -y install wget

#use bash shell
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get -y install gazebo9 ros-$ROS_DISTRO-gazebo-plugins python-pytest ros-$ROS_DISTRO-turtlebot3 ros-$ROS_DISTRO-turtlebot3-simulations ros-$ROS_DISTRO-rviz


# create catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    catkin_init_workspace && \
    cd /root/catkin_ws && \
    catkin_make && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    source "/root/catkin_ws/devel/setup.bash"


# setup entrypoint
#COPY ./ros_entrypoint.sh /
#ENTRYPOINT ["/ros_entrypoint.sh"]
#ENTRYPOINT ["source", "/root/catkin_ws/devel/setup.bash"]

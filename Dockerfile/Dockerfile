ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get upgrade -y

COPY . /home/ros2_ws/src/ros2_control_demos

RUN cd /home/ros2_ws/src \
    && vcs import < ros2_control_demos/ros2_control_demos.${ROS_DISTRO}.repos \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}

RUN cd /home/ros2_ws/ \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --merge-install

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch ros2_control_demo_example_2 diffbot.launch.py start_rviz:=false

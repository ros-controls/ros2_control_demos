ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get upgrade -y \
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/ros2_ws/src/ros2_control_demos

RUN cd /home/ros2_ws/src \
    && rosdep update --rosdistro ${ROS_DISTRO}  \
    && apt-get update \
    && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2\
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

RUN cd /home/ros2_ws/ \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --merge-install --symlink-install

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch ros2_control_demo_example_1 rrbot.launch.py start_rviz:=false

ARG ROS_DISTRO=rolling

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get upgrade -y

COPY . /home/ros2_ws/src/ros2_control_demos

RUN cd /home/ros2_ws/src \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} --skip-keys rviz2

RUN cd /home/ros2_ws/ \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --merge-install --symlink-install

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch ros2_control_demo_example_1 rrbot.launch.py start_rviz:=false

:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_18/doc/userdoc.rst

Example 18: Linux SocketCAN differential-drive hardware
========================================================

This example connects a ``ros2_control`` differential-drive system to a Linux
SocketCAN interface. The raw CAN socket is non-blocking and the transport is
kept separate from the hardware plugin so tests do not require a CAN adapter.

The command frame is ``0x201`` and the feedback frame is ``0x181``. Each frame
contains two little-endian signed 32-bit wheel velocities in milliradians per
second. Configure the interface before starting the launch file, for example:

.. code-block:: shell

   sudo modprobe vcan
   sudo ip link add dev vcan0 type vcan
   sudo ip link set vcan0 up
   ros2 launch ros2_control_demo_example_18 diffbot.launch.xml can_interface:=vcan0

The node does not change CAN bitrate or link state. ``EAGAIN`` means that no
new feedback is available; a failed command write returns ``ERROR`` so the
controller manager can stop the hardware safely.

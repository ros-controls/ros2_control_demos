:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_18/doc/userdoc.rst

Example 18: Linux SocketCAN differential-drive hardware
========================================================

This Linux-only example connects a ``ros2_control`` differential-drive system to a Linux
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
new feedback is available; it does not overwrite the last state. If no valid
feedback arrives for ``feedback_timeout_sec`` (0.5 seconds by default), the
hardware returns ``ERROR``. Malformed frames, CAN error frames, non-finite
values, a closed socket, and failed writes are also errors. CAN error frames,
including bus-off notifications, are detected and propagated as hardware errors;
recovery is intentionally left to the operator/system layer. Command values are
rounded and saturated to the signed 32-bit wire range; the plugin rejects
non-finite commands before sending.

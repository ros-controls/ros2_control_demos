# Example 18: Linux SocketCAN differential drive

This Linux-only example connects a `ros2_control` differential-drive system to a Linux
SocketCAN interface. It uses a non-blocking raw CAN socket and keeps transport
code separate from the hardware plugin so the wire format can be tested without
real motors.

Create a virtual CAN device for a smoke test:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
ros2 launch ros2_control_demo_example_18 diffbot.launch.xml can_interface:=vcan0
```

The example sends command frame `0x201` and accepts feedback frame `0x181`.
Each frame contains two little-endian signed int32 wheel velocities in
milliradians per second. Replace the transport or IDs for a real motor drive.

The socket is opened non-blocking. `EAGAIN` is treated as “no new feedback”;
the hardware returns `ERROR` after `feedback_timeout_sec` (0.5 seconds by
default) rather than integrating stale wheel velocity forever. Malformed data,
CAN error frames, non-finite values, a closed socket, and failed command writes
also return `ERROR` to `controller_manager`, allowing the hardware to be
stopped safely. CAN bitrate and bus-up configuration remain an operator
responsibility and are intentionally not run by the node.

Command values are rounded to milliradians/sec and saturated to the signed
32-bit wire range. Non-finite command values encode as zero in the transport,
but the hardware plugin rejects them before sending. CAN error frames, including
bus-off notifications, are detected and propagated as hardware errors; recovery
is intentionally left to the operator/system layer.

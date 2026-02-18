Design
======

.. contents::
   :depth: 2

Overview
--------

This demo controls an Open Duck Mini robot to walk using velocity commands. The system uses an ONNX machine learning model to generate joint position commands for locomotion.

Control pipeline (three stages):

1. Observation Formatter: Formats sensor data (IMU, joint states, velocity commands, feet contacts) into an observation vector matching the ONNX model's expected input format.

2. ONNX Model Inference: Runs the trained policy model to generate raw action outputs.

3. Action Formatter: Processes the model outputs by scaling, clamping to joint limits, applying rate limiting, and blending with reference motions before sending commands to hardware.

Timing: 50 Hz (0.02s period) with simulation timestep 0.002s, yielding implicit decimation of 10 simulation steps per control update (matches reference implementation).

Architecture
------------

Components
~~~~~~~~~~

1. DuckMiniMujocoSystemInterface: Custom hardware interface extending mujoco_ros2_control to add foot contact detection via MuJoCo collision data.

2. state_interfaces_broadcaster: Aggregates hardware state interfaces (IMU, joint states, contact sensors) into a control_msgs/Float64Values topic.

3. MotionController: Subscribes to sensor data and velocity commands, formats observations, runs ONNX inference, writes joint position commands to hardware.

Data Flow
~~~~~~~~~

.. code-block:: text

   [MuJoCo Simulator]
       ↓ (joint states, IMU data, contact sensors)
   [Hardware Interface]
       ↓ (state interfaces)
   [State Interfaces Broadcaster]
       ↓ (ROS2 topic: /state_interfaces_broadcaster/values)
   [Motion Controller]
       ↑ (ROS2 topic: /motion_controller/cmd_velocity_with_head - VelocityCommandWithHead)
       ↓ (ONNX inference)
       ↓ (command interfaces - joint position commands)
   [Hardware Interface]
       ↓ (joint commands)
   [MuJoCo Simulator]

Hardware Layer
--------------

The hardware interface exposes state interfaces for IMU, joint states, and contact sensors, and accepts command interfaces for joint positions.

Contact Detection (DuckMiniMujocoSystemInterface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The base MujocoSystemInterface supports only "imu" and "fts" sensor types. Contact detection is implemented in this demo's hardware wrapper.

- Registration (on_init): Scans hardware_info.sensors for ``mujoco_type="contact"``. Parses ``body1_name``, ``body2_name`` (required), and optional ``contact_consumer`` ("collision" or "gait"), ``debounce_on_steps``, ``debounce_off_steps``. Body IDs resolved lazily on first read.

- State interfaces: Per sensor, exports ``contact_raw`` (unfiltered 0/1) and ``contact`` (filtered for gait). Xacro: ``<state_interface name="contact_raw"/>``, ``<state_interface name="contact"/>``.

- Update (read): Iterates ``mjData->contact[]``, checks for (body1, body2) or (body2, body1) match. For gait mode, debounce filter: contact=1 after ``debounce_on_steps`` consecutive raw contacts, contact=0 after ``debounce_off_steps`` consecutive no-contacts.

- Xacro: left_foot_contact (foot_assembly, floor), right_foot_contact (foot_assembly_2, floor), both contact_consumer=gait. Optional debounce params per sensor.

MuJoCo Model
~~~~~~~~~~~~

Uses ``open_duck_mini_v2.xml`` with BAM-identified actuator parameters (damping, armature, frictionloss) matching ONNX training. Loaded via ``scene.xml``. STS3215 defaults: damping=0.56, armature=0.027, frictionloss=0.068, kp=13.37, forcerange=-3.23..3.23 N⋅m. MuJoCo built-in position actuators.

Motion Controller
-----------------

Update Rate
~~~~~~~~~~~

50 Hz (0.02s period). Reference uses explicit decimation: 10 sim steps (0.002s each) per control update. mujoco_ros2_control achieves same implicitly: 0.02s / 0.002s = 10 steps. Actions held constant for ~10 sim steps. Motor velocity limit: ``max_change = max_motor_velocity * dt = 5.24 * 0.02 = 0.1048 rad/period``.

Inputs
~~~~~~

- state_interfaces_broadcaster/values: IMU (orientation, gyro, accel), joint positions/velocities, foot contacts (left_foot_contact/contact, right_foot_contact/contact) when use_contact_sensors=true.

- velocity_command_topic: VelocityCommandWithHead (base_velocity + head_commands).

- Previous action (internal).

When use_contact_sensors=false, phase-based synthetic contacts (sin/cos of imitation phase) are used instead.

Processing
~~~~~~~~~~

Format observation vector, run ONNX inference, process outputs (scale, clamp, rate limit), write joint position commands to hardware.

Observation Vector Format
~~~~~~~~~~~~~~~~~~~~~~~~~

Order (ref: v2_rl_walk_mujoco.py, mujoco_infer_base.py):

1. Gyro (3D): angular velocity, optional deadband
2. Accelerometer (3D): linear accel, x-bias 1.3, z optionally flipped
3. Velocity commands (7D): lin_vel_x, lin_vel_y, ang_vel_z, neck_pitch, head_pitch, head_yaw, head_roll
4. Joint positions (N): relative to default
5. Joint velocities (N): scaled
6. Last action (N)
7. Last-last action (N)
8. Last-last-last action (N)
9. Motor targets (N)
10. Feet contacts (2): left, right
11. Imitation phase (2): cos, sin

Total: 17 + 6*N (N = 14 for legs+head).

User Command Interface
----------------------

Topic: ~/cmd_velocity_with_head (default). Message: ``example_18_motion_controller_msgs/VelocityCommandWithHead`` with ``base_velocity`` (Twist) and ``head_commands`` (float64[4]: neck_pitch, head_pitch, head_yaw, head_roll in rad).

Example: Forward 0.5 m/s — base_velocity.linear.x=0.5, linear.y=0, angular.z=0, head_commands=[0,0,0,0]. Observation slice: [0.5, 0, 0, 0, 0, 0, 0].

Controller Manager
------------------

Runs control loop at 50 Hz. Manages state_interfaces_broadcaster and MotionController lifecycle.

# ros2_control_demo_example_18

This example demonstrates ONNX policy-driven locomotion for the Open Duck Mini robot in MuJoCo simulation. The robot follows velocity commands using an ML policy that generates joint position commands. The demo includes a custom hardware interface (``DuckMiniMujocoSystemInterface``) that extends the MuJoCo interface to add foot contact detection for gait control.

Find the full documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_18/doc/userdoc.html).

# ros2_control_demo_example_21

This example runs the Open Duck Mini walking policy of example_18 from a PyTorch model exported with ExecuTorch. The controller loads the `.pte` program once in `on_configure()`, and the policy call inside `update()` does not allocate heap memory. The robot, simulation, message and observation handling come from example_18.

Find the full documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_21/doc/userdoc.html).

# Using admittance controller

This manual targets ROS2 rolling.

1. Create a new ROS workspace and compile it with ROS2 rolling.
1. Install some dependencies
   ```
   sudo apt-get install python3-colcon-common-extensions python3-rosdep python3-vcstool
   sudo rosdep init
   rosdep update
   ```
1. Checkout the repositories from `admittance_controller.repos` file:
   ```
   wget https://raw.githubusercontent.com/destogl/ros2_control_demos/admittance-controller-setup/admittance_controller.repos
   vcs import --input admittance_controller.repos .
   rosdep install --from-paths . -y -i
   ```

1. Compile your workspace using `colcon build`.
   NOTE: Maybe you get some error message in output. Please ignore those until we fix the dependencies.

1. Start the demo using:
   ```
   ros2 launch ros2_control_demo_bringup admittance_controller_demo.launch.py
   ```

1. Listing controller with `ros2 control list_controllers` you should see admittance controller in the state `inactive`.

1. Activate the admittance controller:
   ```
   ros2 control switch_controllers --start admittance_controller
   ```

1. Start custom version of `teleop_twist_keyboard` to publish fake forces:
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

1. Use keyboard keys to impose fake forces on the robot.
   TIP: set maximal "speed" values (using "q" key) around `10` to get visible force/torque influence. To influence the robot with separate forces and torques use "holonomic"-mode (hold key or turn on Caps Lock). For further adjustment follow the notes in terminal.

   To see the faked forces use:
   ```
   ros2 topic echo /faked_forces_controller/commands
   ```

## Features

### Dynamic parameters

Admittance controller provides a possibility to set and change its parameters during run-time.
This is useful for tuning its parameters or using different set of parameters for different scenarios.
It is recommended to reactivate (deactivate then activate) the controller each time when parameters are adjusted.
Otherwise, the controller could become unstable when updating multiple parameters.
Nevertheless, you can configure updating parameters without reactivation using `enable_parameter_update_without_reactivation`.

1. To test dynamic parameters set first check how the robot behaves in 'X'-direction using "<Shift>+I" and "<Shift>+K" keys in the `teleop_twist_keyboard` node.

1. Change a parameter of admittance controller, e.g., damping ratio:
   ```
   ros2 param set /admittance_controller admittance.damping_ratio.y 20
   ```

1. Then restart controller:
   ```
   ros2 control switch_controllers --stop admittance_controller --start admittance_controller
   ```

1. Then you should see how the controllers dynamic has changed when executing the same movements.

### Using Filters

Admittance controller uses filter chain to filter input force data.
The usual example is to compensate the gravitational influence of a tool.
In the example files there are already two filters for Gravity compensation.

The GravityCompensator filter compensate the gravitational influence to force measurements. (TBD: add link to the filter's documentation).

To test the filter, start the admittance controller and then set filter's force parameter:
  ```
  ros2 param set /admittance_controller input_wrench_filter_chain.filter1.params.force 5.0

  ```
NOTE: if using fake hardware you should first publish some force using `teleop_twist_keyboard` and then will this work (this will reset values from NaN).


## Test with URSim (not finished!)

1. Build and run URSim as described [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#usage-with-official-ur-simulator)
   NOTE: First time docker container has to be build and everything has to be compiled, so take a cup of coffee or your favorite tea and start reading your emails :D

1. Stop `ursim_driver_driver_*` docker with `docker stop ...`

1. Start demo setup with the following command:
   ```
   ros2 launch ros2_control_demo_bringup admittance_controller_demo.launch.py use_fake_hardware:=false fake_sensor_commands:=false headless_mode:=true robot_ip:=192.168.56.101 launch_dashboard_client:=true
   ```

1. Now everything should start normally

1. Simulate forces on FTS... TBD... (maybe not possible)

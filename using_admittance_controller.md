# Using admittance controller

This manual targets ROS2 rolling.

1. Create a new ROS workspace and navigate to the src folder.
1. Install some dependencies
   ```
   sudo apt-get install python3-colcon-common-extensions python3-rosdep python3-vcstool
   sudo rosdep init
   rosdep update
   ```
1. Clone this moveit2 repository and import the `moveit2/moveit2.repos` file:
   ```
   git clone https://github.com/pac48/moveit2.git -b pr-support-chained-controllers
   vcs import < "moveit2/moveit2.repos"
   ```
1. Checkout the repositories from `admittance_controller.repos` file:
   ```
   wget https://raw.githubusercontent.com/pac48/ros2_control_demos/add-admittance-controller/admittance_controller.repos
   vcs import --input admittance_controller.repos .
   rosdep install --from-paths . -y -i --ignore-src
   ```
   
1. Compile your workspace using `colcon build`.

###  Demo: manual activation
Start the demo using:
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
   
###  Demo: MoveIt2
Start the demo using:
   ```
   ros2 launch ros2_control_demo_bringup admittance_controller_demo.launch.py
   ```

1. Listing controller with `ros2 control list_controllers` you should see admittance controller in the state `inactive`.

1. Start custom version of `teleop_twist_keyboard` to publish fake forces:
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
1. Create set a random valid goal in RVIZ and press plan & execute.
1. Use the keyboard to impose force on the robot while it is moving.

## Features

### Dynamic parameters

Admittance controller provides a possibility to set and change its parameters during run-time.
This is useful for tuning its parameters or using different set of parameters for different scenarios.
It is recommended to reactivate (deactivate then activate) the controller each time when parameters are adjusted.
Otherwise, the controller could become unstable when updating multiple parameters.
Nevertheless, you can configure updating parameters without reactivation using `enable_parameter_update_without_reactivation`.

1. To test dynamic parameters set first check how the robot behaves in 'X'-direction using "<Shift>+I" and "<Shift>+K" keys in the `teleop_twist_keyboard` node.

1. Change a parameter of admittance controller, e.g., stiffness:
   ```
   ros2 param set /admittance_controller admittance.stiffness [5.0,5.0,5.0,5.0,1.0,1.0,1.0]
   ```

1. Then you should see how the controllers dynamic has changed when executing the same movements.

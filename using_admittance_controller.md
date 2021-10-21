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
   TBD
   ```

:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/doc/index.rst

.. _ros2_control_demos:

#################
Demos
#################

This `GitHub Repository <https://github.com/ros-controls/ros2_control_demos>`_
provides templates for the development of ``ros2_control``-enabled robots and simple simulations to demonstrate and prove ``ros2_control`` concepts.

If you want to have a rather step by step manual how to do things with ``ros2_control`` checkout `ros-control/roscon2022_workshop <https://github.com/ros-controls/roscon2022_workshop>`_ repository.

==========================================
What you can find in this repository
==========================================

This repository demonstrates the following ``ros2_control`` concepts:

  * Creating a ``HardwareInterface`` for a System, Sensor, and Actuator.
  * Creating a robot description in the form of URDF files.
  * Loading the configuration and starting a robot using launch files.
  * Control of a differential mobile base *DiffBot*.
  * Control of two joints of *RRBot*.
  * Control of a 6-degrees-of-freedom robot.
  * Implementing a controller switching strategy for a robot.
  * Using joint limits and transmission concepts in ``ros2_control``.

=====================
Goals
=====================

The repository has two other goals:

1. Implements the example configuration described in the ``ros-controls/roadmap`` repository file `components_architecture_and_urdf_examples <https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md>`_.
2. The repository is a validation environment for ``ros2_control`` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).

=====================
Examples Overview
=====================

Example 1: RRBot
   *RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.

Example 2: DiffBot
   *DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
   The robot is basically a box moving according to differential drive kinematics.

Example 3: "RRBot with multiple interfaces"
   *RRBot* with multiple interfaces.

Example 4: "Industrial robot with integrated sensor"
   *RRBot* with an integrated sensor.

Example 5: "Industrial robot with externally connected sensor"
   *RRBot* with an externally connected sensor.

Example 6: "Modular robot with separate communication to each actuator"
   The example shows how to implement robot hardware with separate communication to each actuator.

Example 7: "6-DOF robot"
   A full tutorial for a 6 DOF robot for intermediate ROS 2 users.

Example 8: "Using transmissions"
   *RRBot* with an exposed transmission interface.

Example 9: "Gazebo Classic"
   Demonstrates how to switch between simulation and hardware.

Example 10: "GPIO interfaces"
   Industrial robot with GPIO interfaces

Example 11: "CarlikeBot"
    *CarlikeBot* with a bicycle steering controller

Example 12: "Controller chaining"
   The example shows a simple chainable controller and its integration to form a controller chain to control the joints of *RRBot*.

Example 13: "Multi-robot system with hardware lifecycle management"
   This example shows how to handle multiple robots in a single controller manager instance.

Example 14: "Modular robots with actuators not providing states and with additional sensors"
   The example shows how to implement robot hardware with actuators not providing states and with additional sensors.

Example 15: "Using multiple controller managers"
   This example shows how to integrate multiple robots under different controller manager instances.


.. _ros2_control_demos_install:

=====================
Installation
=====================

You can install the demos locally or use the provided docker file.


Local installation
------------------

If you have ROS 2 installed already, choose the right version of this documentation and branch of the ``ros2_control_demos`` repository matching you ROS 2 distribution, see `this table <https://github.com/ros-controls/ros2_control_demos#build-status>`__.

Otherwise, install `ROS 2 {DISTRO} on your computer <https://docs.ros.org/en/{DISTRO}/Installation.html>`__.

.. note::

  ``ros2_control`` and ``ros2_controllers`` packages are released and can be installed using a package manager.
  We provide officially released and maintained debian packages, which can easily be installed via aptitude.
  However, there might be cases in which not-yet released demos or features are only available through a source build in your own workspace.

Build from debian packages
^^^^^^^^^^^^^^^^^^^^^^^^^^

Download the ``ros2_control_demos`` repository and install its dependencies with

.. code-block:: shell

  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://github.com/ros-controls/ros2_control_demos -b {REPOS_FILE_BRANCH}
  cd ~/ros2_ws/
  rosdep update --rosdistro=$ROS_DISTRO
  sudo apt-get update
  sudo rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}

Now you can build the repository (source your ROS 2 installation first)

.. code-block:: shell

  cd ~/ros2_ws/
  . /opt/ros/${ROS_DISTRO}/setup.sh
  colcon build --merge-install


Build from source
^^^^^^^^^^^^^^^^^

* Download all repositories

  .. code-block:: shell

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/ros-controls/ros2_control_demos
    cd ~/ros2_ws/
    vcs import src < src/ros2_control_demos/ros2_control_demos.$ROS_DISTRO.repos
    rosdep update --rosdistro=$ROS_DISTRO
    sudo apt-get update

* Install dependencies:

  .. code-block:: shell

    rosdep install --from-paths src --ignore-src -r -y

* Build everything, e.g. with:

  .. code-block:: shell

    . /opt/ros/${ROS_DISTRO}/setup.sh
    colcon build --symlink-install

* Do not forget to source ``setup.bash`` from the ``install`` folder!

Using Docker
---------------

First, build the dockerfile with

.. code-block:: shell

  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://github.com/ros-controls/ros2_control_demos
  cd ros2_control_demos
  docker build . -t ros2_control_demos -f Dockerfile/Dockerfile

To view the robot
^^^^^^^^^^^^^^^^^

Docker now allows us to run the demo without the GUI if configured properly. Now we can view the robot by the following procedure:

After having `ROS 2 installed <https://docs.ros.org/en/humble/Installation.html>`__ on your local system (not inside the docker),  we can use ``rviz2`` to visualize the robot state and ``joint_state_publisher_gui`` package to give manual joint values to the robot. To install the package you can run:

.. code-block:: shell

  sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-rviz2

Then we are ready to bring up all the components to view the robot. Let's start with the docker container by running the following command:

.. code-block:: shell

  docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_1 view_robot.launch.py gui:=false

.. note::

  Depending on your machine settings, it might be possible that you have to omit ``--net host``.

Now, we need to start ``rviz2`` to view the robot as well as ``joint_state_publisher_gui``, each in their own terminals after sourcing our ROS 2  installation.

Terminal 1:

.. code-block:: shell

  source /opt/ros/${ROS_DISTRO}/setup.bash
  ros2 run joint_state_publisher_gui joint_state_publisher_gui

Terminal 2:

.. code-block:: shell

  source /opt/ros/${ROS_DISTRO}/setup.bash
  cd ~/ros2_ws
  rviz2 -d src/ros2_control_demos/example_1/description/rviz/rrbot.rviz

Now, you can see the robot moving by changing the values of the joints by moving the sliders around in the ``joint_state_publisher_gui``.

To run the ros2_control demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following command runs the demo without the GUI:

.. code-block:: shell

  docker run -it --rm --name ros2_control_demos --net host ros2_control_demos

.. note::

  Depending on your machine settings, it might be possible that you have to omit ``--net host``.

Then on your local machine, you can run rviz2 with the config file specified:

.. code-block:: shell

  cd ~/ros2_ws
  source /opt/ros/${ROS_DISTRO}/setup.sh
  rviz2 -d src/ros2_control_demos/example_1/description/rviz/rrbot.rviz

You can also run other commands or launch files from the docker, e.g.

.. code-block:: shell

  docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_2 diffbot.launch.py

=====================
Quick Hints
=====================

These are some quick hints, especially for those coming from a ROS1 control background:

  * There are now three categories of hardware components: *Sensor*, *Actuator*, and *System*.
    *Sensor* is for individual sensors; *Actuator* is for individual actuators; *System* is for any combination of multiple sensors/actuators.
    You could think of a Sensor as read-only.
    All components are used as plugins and therefore exported using ``PLUGINLIB_EXPORT_CLASS`` macro.
  * *ros(1)_control* only allowed three hardware interface types: position, velocity, and effort.
    *ros2_control* allows you to create any interface type by defining a custom string. For example, you might define a ``position_in_degrees`` or a ``temperature`` interface.
    The most common (position, velocity, acceleration, effort) are already defined as constants in hardware_interface/types/hardware_interface_type_values.hpp.
  * In ros2_control, all parameters for the driver are specified in the URDF.
    The ros2_control framework uses the **<ros2_control>** tag in the URDF.
  * Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.

=====================
Examples
=====================
.. toctree::
   :titlesonly:

   Example 1: RRBot <../example_1/doc/userdoc.rst>
   Example 2: DiffBot <../example_2/doc/userdoc.rst>
   Example 3: RRBot with multiple interfaces <../example_3/doc/userdoc.rst>
   Example 4: Industrial robot with integrated sensor <../example_4/doc/userdoc.rst>
   Example 5: Industrial robots with externally connected sensor <../example_5/doc/userdoc.rst>
   Example 6: Modular robots with separate communication to each actuator <../example_6/doc/userdoc.rst>
   Example 7: Full tutorial with a 6DOF robot <../example_7/doc/userdoc.rst>
   Example 8: Using transmissions <../example_8/doc/userdoc.rst>
   Example 9: Gazebo classic <../example_9/doc/userdoc.rst>
   Example 10: Industrial robot with GPIO interfaces <../example_10/doc/userdoc.rst>
   Example 11: CarlikeBot <../example_11/doc/userdoc.rst>
   Example 12: Controller chaining <../example_12/doc/userdoc.rst>
   Example 13: Multiple robots <../example_13/doc/userdoc.rst>
   Example 14: Modular robots with actuators not providing states <../example_14/doc/userdoc.rst>
   Example 15: Using multiple controller managers <../example_15/doc/userdoc.rst>

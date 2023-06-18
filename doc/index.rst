:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/doc/index.rst

.. _ros2_control_demos:

#################
Demos
#################

This `GitHub Repository <https://github.com/ros-controls/ros2_control_demos>`_
provides templates for the development of ros2_control-enabled robots and a simple simulations to demonstrate and prove ros2_control concepts.

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
  * Implementing a controller switching strategy for a robot.
  * Using joint limits and transmission concepts in ``ros2_control``.

=====================
Goals
=====================

The repository has two other goals:

1. Implements the example configuration described in the ``ros-controls/roadmap`` repository file `components_architecture_and_urdf_examples <https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md>`_.
2. The repository is a validation environment for ``ros2_control`` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).

=====================
Example Overview
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


Example 5: "Industrial Robots with externally connected sensor"
   *RRBot* with an externally connected sensor.


Example 6: "Modular Robots with separate communication to each actuator"
   The example shows how to implement robot hardware with separate communication to each actuator.


Example 8: "Using transmissions"
   *RRBot* with an exposed transmission interface.


Example 9: "Gazebo Classic"
   Demonstrates how to switch between simulation and hardware.


=====================
Installation
=====================

You can install the demos manually or use the provided docker file.

Manual Install
---------------

First, you have to install `ROS 2 on your computer <https://docs.ros.org/en/rolling/Installation.html>`__.

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
  git clone https://github.com/ros-controls/ros2_control_demos
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

Docker now allows us to run the demo without the GUI if configured properly. The following command runs the demo without the GUI:

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
  * Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.
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
   Example 5: Industrial Robots with externally connected sensor <../example_5/doc/userdoc.rst>
   Example 6: Modular Robots with separate communication to each actuator <../example_6/doc/userdoc.rst>
   Example 8: Using transmissions <../example_8/doc/userdoc.rst>
   Example 9: Gazebo Classic <../example_9/doc/userdoc.rst>

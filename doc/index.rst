.. _ros2_control_demos:

Demos
-----

This repository provides templates for the development of ros2_control-enabled robots and a simple simulations to demonstrate and prove ros2_control concepts.

Repository organization
^^^^^^^^^^^^^^^^^^^^^^^

**Note:** the following list is comprehensive.

The repository is organized in the following packages:

  - ``ros2_control_demo_hardware`` - implementation of example hardware interfaces,
  - ``ros2_control_demo_bringup`` - nodes starting hardware interfaces, controllers and GUIs.
  - ``ros2_control_test_node`` - nodes for testing ros2_control-enabled robots and their integration in the framework.

Mode switching demo
^^^^^^^^^^^^^^^^^^^

Start up the multi interface rrbot system:

.. code-block:: bash

    ros2 launch ros2_control_demo_bringup rrbot_system_multi_interface.launch.py

List the available interfaces

.. code-block:: bash

    $ ros2 control list_hardware_interfaces
    command interfaces
        joint1/acceleration [unclaimed]
        joint1/position [unclaimed]
        joint1/velocity [unclaimed]
        joint2/acceleration [unclaimed]
        joint2/position [unclaimed]
        joint2/velocity [unclaimed]
    state interfaces
         joint1/acceleration
         joint1/position
         joint1/velocity
         joint2/acceleration
         joint2/position
         joint2/velocity

Load and configure all controllers

.. code-block:: bash

    ros2 control load_controller forward_command_controller_position --state configure
    ros2 control load_controller forward_command_controller_velocity --state configure
    ros2 control load_controller forward_command_controller_acceleration --state configure
    ros2 control load_controller forward_command_controller_illegal1 --state configure
    ros2 control load_controller forward_command_controller_illegal2 --state configure
    ros2 control load_controller joint_state_controller --state configure


Start the position controller

.. code-block:: bash

    ros2 control set_controller_state forward_command_controller_position start

Check the hardware interfaces, position interfaces should be claimed now

.. code-block:: bash

    $ ros2 control list_hardware_interfaces
    command interfaces
        joint1/acceleration [unclaimed]
        joint1/position [claimed]
        joint1/velocity [unclaimed]
        joint2/acceleration [unclaimed]
        joint2/position [claimed]
        joint2/velocity [unclaimed]
    state interfaces
         joint1/acceleration
         joint1/position
         joint1/velocity
         joint2/acceleration
         joint2/position
         joint2/velocity

Let's switch controllers now to velocity

.. code-block:: bash

    ros2 control switch_controllers --stop-controllers forward_command_controller_position --start-controllers forward_command_controller_velocity

List hardware interfaces again to see that indeed position interfaces have been unclaimed while velocity is claimed now

.. code-block:: bash

    $ ros2 control list_hardware_interfaces
    command interfaces
        joint1/acceleration [unclaimed]
        joint1/position [unclaimed]
        joint1/velocity [claimed]
        joint2/acceleration [unclaimed]
        joint2/position [unclaimed]
        joint2/velocity [claimed]
    state interfaces
         joint1/acceleration
         joint1/position
         joint1/velocity
         joint2/acceleration
         joint2/position
         joint2/velocity

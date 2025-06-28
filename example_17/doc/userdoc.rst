:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

Example 17: RRBot with Hardware Component that publishes diagnostics
=====================================================================

This example shows how to publish diagnostics from a hardware component using the Executor passed from Controller Manager.

It is essentially the same as Example 1, but with a modified hardware interface plugin that uses the aforementioned Executor and adds its own ROS 2 node to publish diagnostics.

See the :ref:`Implementation Details of the Diagnostic Publisher <diagnostic_publisher_implementation>` for more information.

For *example_17*, the hardware interface plugin is implemented having only one interface.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.
* Examples: KUKA RSI

The *RRBot* URDF files can be found in the ``description/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------
Follow the same basic steps as in Example 1. You can find the details here:
:ref:`Example 1: RRBot System Position Only <ros2_control_demos_example_1_userdoc>`.

This tutorial differs by including a hardware interface that publishes diagnostics.

A custom ROS 2 node is created inside the hardware interface to publish status messages. This node is added to the executor provided by the controller manager.

The node:

- Is named ``<hardware_name>_status_publisher``, where slashes are replaced by underscores.
- Publishes on the topic ``/rrbot_internal_status``.
- Uses message type ``std_msgs/msg/String``.
- Sends a message every 2 seconds indicating the robot is alive.

To check that the node is running and diagnostics are published correctly:

.. tabs::

   .. group-tab:: Local

      .. code-block:: shell

         # List available nodes
         ros2 node list

         # You should see something like:
         # /rrbot_status_publisher

         # List topics and confirm diagnostics topic is available
         ros2 topic list

         # Confirm message type of the diagnostics topic
         ros2 topic info /rrbot_internal_status

         # Echo the messages published by the diagnostics node
         ros2 topic echo /rrbot_internal_status

   .. group-tab:: Docker

      Enter the running container shell first:

      .. code-block:: shell

         docker exec -it ros2_control_demos ./entrypoint.sh bash

      Then run the same commands inside the container:

      .. code-block:: shell

         ros2 node list
         ros2 topic list
         ros2 topic info /rrbot_internal_status
         ros2 topic echo /rrbot_internal_status

The echoed message should look similar to:

.. code-block:: shell

   data: "RRBot 'RRBot' is alive at 1751087597.146549"

.. note::

   The diagnostics node and its timer are created only if the executor is successfully passed to the hardware component. If you don't see the topic or node, ensure the hardware plugin is correctly implemented and that the controller manager is providing an executor.

.. _diagnostic_publisher_implementation:

Implementation Details of the Diagnostic Publisher
--------------------------------------------------

This example builds upon the standard ``RRBot`` hardware interface by demonstrating the suggested way for a hardware component to run its own ROS 2 node for tasks like publishing diagnostics. This is achieved by leveraging the executor that is managed by the ``ControllerManager``.

The key steps implemented in the ``rrbot.cpp`` hardware interface are:

1.  **Receiving the Executor Reference**: The ``on_init`` method of the hardware interface is implemented with an updated signature that accepts ``HardwareComponentInterfaceParams``. This struct contains a weak pointer to the ``ControllerManager``'s executor.

    .. code-block:: cpp

      // Get Weak Pointer to Executor from HardwareComponentInterfaceParams
      executor_ = params.executor;

2.  **Safely Accessing the Executor**: Before using the executor, its ``weak_ptr`` must be "locked" into a ``shared_ptr``. This is a crucial safety check to ensure the executor is still valid. The C++17 ``if-initializer`` syntax provides a clean and standard way to do this.

    .. code-block:: cpp

      if (auto locked_executor = executor_.lock())
      {
        // ... executor is valid and can be used here ...
      }
      else
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

3.  **Creating and Adding a Node**: Inside the locked scope, a standard ``rclcpp::Node`` is created. This new node is then added to the ``ControllerManager``'s executor. This allows the node's callbacks (e.g., for timers or subscriptions) to be processed by the same multi-threaded executor that handles the ``ControllerManager``'s services.

    .. code-block:: cpp

      // Inside the `if (auto locked_executor = ...)` block
      status_node_ = std::make_shared<rclcpp::Node>(info_.name + "_status_publisher");
      locked_executor->add_node(status_node_->get_node_base_interface());


4.  **Publishing Diagnostics**: A publisher and a periodic wall timer are created on the new node. The timer's callback periodically publishes a simple status message, demonstrating that the hardware component's internal node is alive and running alongside the main control loop.

    .. code-block:: cpp

      // Inside the `if (auto locked_executor = ...)` block
      status_publisher_ =
        status_node_->create_publisher<std_msgs::msg::String>("rrbot_internal_status", 10);

      status_timer_ = status_node_->create_wall_timer(
        2s, [this](){ /* ... lambda to publish message ... */ });

This pattern is the recommended approach for hardware components that need to perform their own ROS communications without interfering with the real-time control loop.

Files used for this demos
-------------------------

* Launch file: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/bringup/launch/rrbot.launch.py>`__
* Controllers yaml:

  * `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/bringup/config/rrbot_controllers.yaml>`__
  * `rrbot_jtc.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/bringup/config/rrbot_jtc.yaml>`__

* URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/description/urdf/rrbot.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/description/ros2_control/rrbot.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__

* Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
  * ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
  * ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): :ref:`doc <forward_command_controller_userdoc>`
  * ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller>`__): :ref:`doc <joint_trajectory_controller_userdoc>`

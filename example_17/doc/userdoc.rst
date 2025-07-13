:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

Example 17: RRBot with Hardware Component that publishes diagnostics
=====================================================================

This example shows how to publish diagnostics from a hardware component using the Executor passed from Controller Manager. It also demonstrates the use of a default node provided to the hardware component.

It is essentially the same as Example 1, but with a modified hardware interface plugin that uses the aforementioned Executor to add its own custom ROS 2 node, and also uses a default node to publish diagnostics.

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

This tutorial differs by including a hardware interface that publishes diagnostics from two different nodes: a "default" node and a "custom" node.

1.  A **default node** is automatically provided to the hardware component. This node will publish standard ROS 2 diagnostic messages.
2.  A **custom ROS 2 node** is created inside the hardware interface and added to the executor. This node will publish a simpler ``std_msgs/msg/String`` status message.

The nodes:

- Default node:
  - Is named after the hardware component (e.g., ``/RRBotSystemPositionOnlyHardware``).
  - Publishes on the topic ``/diagnostics``.
  - Uses message type ``diagnostic_msgs/msg/DiagnosticArray``.
  - Sends a message every 2.5 seconds.
- Custom node:
  - Is named ``<hardware_name>_custom_node`` (e.g., ``/rrbot_custom_node``).
  - Publishes on the topic ``/rrbot_custom_status``.
  - Uses message type ``std_msgs/msg/String``.
  - Sends a message every 2 seconds.

To check that the nodes are running and diagnostics are published correctly:

.. tabs::

   .. group-tab:: Local

      .. code-block:: shell

         # List available nodes
         ros2 node list

         # You should see something like:
         # /rrbot           (the default node)
         # /rrbot_custom_node (the custom node)
         # /controller_manager
         # /robot_state_publisher

         # List topics and confirm diagnostics topics are available
         ros2 topic list

         # Confirm message type of the diagnostics topic
         ros2 topic info /diagnostics
         # Should show: diagnostic_msgs/msg/DiagnosticArray

         # Confirm message type of the custom status topic
         ros2 topic info /rrbot_custom_status
         # Should show: std_msgs/msg/String

         # Echo the raw messages published by the default node
         ros2 topic echo /diagnostics

         # Echo the messages published by the custom node
         ros2 topic echo /rrbot_custom_status

   .. group-tab:: Docker

      Enter the running container shell first:

      .. code-block:: shell

         docker exec -it ros2_control_demos ./entrypoint.sh bash

      Then run the same commands inside the container:

      .. code-block:: shell

         ros2 node list
         ros2 topic info /diagnostics
         ros2 topic info /rrbot_custom_status
         ros2 topic echo /diagnostics
         ros2 topic echo /rrbot_custom_status

The echoed messages should look similar to:

.. code-block:: yaml

   # From /diagnostics (showing the default node's output)
   header:
     stamp:
       sec: 1678886401
       nanosec: 123456789
   status:
   - level: 0
     name: "RRBot"
     message: "Hardware is OK"
     hardware_id: ""
     values: []
   ---

.. code-block:: shell

   # From /rrbot_custom_status (showing the custom node's output)
   data: RRBot 'RRBot' custom node is alive at 1751087597.146549
   ---

.. note::

   The custom diagnostics node and its timer are created only if the executor is successfully passed to the hardware component. If you don't see the topic or node, ensure the hardware plugin is correctly implemented and that the controller manager is providing an executor.

.. _diagnostic_publisher_implementation:

Implementation Details of the Diagnostic Publisher
--------------------------------------------------

This example builds upon the standard ``RRBot`` hardware interface by demonstrating the suggested ways for a hardware component to run its own ROS 2 nodes for tasks like publishing diagnostics. Two methods are shown: using a pre-existing "default" node and creating a new "custom" node that is added to the ``ControllerManager``'s executor.

The key steps implemented in the ``rrbot.cpp`` hardware interface are:

**1. Using the Default Node (Recommended Method)**

The ``HardwareComponentInterface`` base class provides a ``get_node()`` method, which returns a ``shared_ptr`` to an ``rclcpp::Node``. This node is already configured and spun by the ``ControllerManager``, making it the easiest way to perform ROS communications.

The following example shows how to use this node to publish standard diagnostic messages.

.. code-block:: cpp

  // Get Default Node added to executor
  auto default_node = get_node();
  if (default_node)
  {
    // Create a publisher for diagnostic messages
    default_status_publisher_ =
      default_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // Create a timer to periodically publish the diagnostics
    auto default_timer_callback = [this]() -> void
    {
      if (default_status_publisher_)
      {
        // Create the top-level message
        auto diagnostic_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diagnostic_msg->header.stamp = get_node()->now();
        diagnostic_msg->status.resize(1);

        // Populate the status for this hardware component
        diagnostic_msg->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_msg->status[0].name = this->get_name();
        diagnostic_msg->status[0].message = "Hardware is OK";

        // Publish the message
        default_status_publisher_->publish(std::move(diagnostic_msg));
      }
    };

    using namespace std::chrono_literals;
    default_status_timer_ = default_node->create_wall_timer(2.5s, default_timer_callback);
  }

.. note::
   It is standard practice to publish diagnostic arrays to the ``/diagnostics`` topic so they can be monitored by tools like ``rqt_robot_monitor``. The ``status.name`` field is used to differentiate between different components.

**2. Creating a Custom Node with the Executor (Advanced Method)**

For cases where a separate node identity is required, a hardware component can create its own node and add it to the ``ControllerManager``'s executor.

1.  **Receiving the Executor Reference**: The ``on_init`` method of the hardware interface is implemented with an updated signature that accepts ``HardwareComponentInterfaceParams``. This struct contains a weak pointer to the ``ControllerManager``'s executor.

    .. code-block:: cpp

      // Get Weak Pointer to Executor from HardwareComponentInterfaceParams
      executor_ = params.executor;

2.  **Safely Accessing the Executor**: Before using the executor, its ``weak_ptr`` must be "locked" into a ``shared_ptr``. This is a crucial safety check to ensure the executor is still valid.

    .. code-block:: cpp

      if (auto locked_executor = executor_.lock())
      {
        // ... executor is valid and can be used here ...
      }
      else
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

3.  **Creating and Adding a Node**: Inside the locked scope, a standard ``rclcpp::Node`` is created. This new node is then added to the ``ControllerManager``'s executor. This allows the node's callbacks (e.g., for timers or subscriptions) to be processed by the same multi-threaded executor.

    .. code-block:: cpp

      // Inside the `if (auto locked_executor = ...)` block
      custom_status_node_ = std::make_shared<rclcpp::Node>(info_.name + "_custom_node");
      locked_executor->add_node(custom_status_node_->get_node_base_interface());


4.  **Publishing Status Messages**: A publisher and a periodic wall timer are created on the new custom node. The timer's callback periodically publishes a status message, demonstrating that the node is alive and running alongside the main control loop.

    .. code-block:: cpp

      // Inside the `if (auto locked_executor = ...)` block
      custom_status_publisher_ =
        custom_status_node_->create_publisher<std_msgs::msg::String>("rrbot_custom_status", 10);

      custom_status_timer_ = custom_status_node_->create_wall_timer(
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

* Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
  * ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
  * ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): :ref:`doc <forward_command_controller_userdoc>`
  * ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller>`__): :ref:`doc <joint_trajectory_controller_userdoc>`

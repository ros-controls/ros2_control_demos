:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

Example 17: RRBot with Hardware Component that publishes diagnostics and status messages
========================================================================================

This example shows how to publish diagnostics and status messages from a hardware component using ROS 2 features available within the ``ros2_control`` framework.

It is essentially the same as Example 1, but with a modified hardware interface plugin that demonstrates two methods for publishing status information:
1.  Using the standard ``diagnostic_updater`` on the default node to publish to the ``/diagnostics`` topic. This is the recommended approach for hardware status reporting.
2.  Using the Controller Manager's Executor to add a custom ROS 2 node for publishing to a separate, non-standard topic.
3.  Using the framework managed default publisher, which published with a ``HardwareStatus`` message, this is the recommended way to approach standard hardware status reporting

See :ref:`Implementation Details of the Diagnostic Publisher <diagnostic_publisher_implementation>` and :ref:`Implementation Details of the Hardware Status Publisher <hardware_status_publisher_implementation>` for more information.

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

This tutorial differs by including a hardware interface that publishes diagnostics using two different mechanisms:

1.  A **default node** is automatically provided to the hardware component. We use the standard ``diagnostic_updater`` library on this node to publish structured diagnostic data.
2.  A **custom ROS 2 node** is created inside the hardware interface and added to the executor provided by the controller manager. This demonstrates a more manual approach useful for non-diagnostic topics.

as well as

1.  A  **default publisher** is automatically created through steps detailed in :ref:`Implementation Details of the Hardware Status Publisher <hardware_status_publisher_implementation>`.

The nodes and topics:

- Default node (via ``diagnostic_updater``):
  - Is named after the hardware component (e.g., ``/RRBot``).
  - Publishes periodically to the standard ``/diagnostics`` topic.
  - Uses message type ``diagnostic_msgs/msg/DiagnosticArray``.
- Custom node:
  - Is named ``<hardware_name>_custom_node`` (e.g., ``/rrbot_custom_node``).
  - Publishes on the topic ``/rrbot_custom_status``.
  - Uses message type ``std_msgs/msg/String``.
  - Sends a message every 2 seconds.
- Default Publisher:
  - Publishes on the topic ``/rrbot/hardware_status``.
  - Uses message type ``control_msgs/msg/HardwareStatus``.
  - Sends a message at rate specified by ``status_publish_rate`` parameter in ros2_control tag.

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

         # Confirm message type of the default publisher topic
         ros2 topic info /rrbot/hardware_status
         # Should show: control_msgs/msg/HardwareStatus

         # Echo the raw messages published by the default node
         ros2 topic echo /diagnostics

         # Echo the messages published by the custom node
         ros2 topic echo /rrbot_custom_status

         # Echo the messages published by default publisher
         ros2 topic echo /rrbot/hardware_status

   .. group-tab:: Docker

      Enter the running container shell first:

      .. code-block:: shell

         docker exec -it ros2_control_demos ./entrypoint.sh bash

      Then run the same commands inside the container:

      .. code-block:: shell

         ros2 node list
         ros2 topic info /diagnostics
         ros2 topic info /rrbot_custom_status
         ros2 topic info /rrbot/hardware_status
         ros2 topic echo /diagnostics
         ros2 topic echo /rrbot_custom_status
         ros2 topic echo /rrbot/hardware_status

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

.. code-block:: shell

   # From /rrbot_custom_status (showing the custom node's output)
   data: RRBot 'RRBot' custom node is alive at 1751087597.146549

.. code-block:: yaml

   # From /rrbot/hardware_status (showing the default publisher's output)
    header:
      stamp:
        sec: 168700
        nanosec: 87768976
      frame_id: ''
    hardware_id: RRBot
    hardware_device_states:
    - header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      device_id: joint1
      hardware_status:
      - health_status: 1
        error_domain: []
        operational_mode: 2
        power_state: 3
        connectivity_status: 0
        manufacturer: ''
        model: ''
        firmware_version: ''
        state_details:
        - key: position_state
          value: '0.000000'
      canopen_states: []
      ethercat_states: []
      vda5050_states: []
    - header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      device_id: joint2
      hardware_status:
      - health_status: 1
        error_domain: []
        operational_mode: 2
        power_state: 3
        connectivity_status: 0
        manufacturer: ''
        model: ''
        firmware_version: ''
        state_details:
        - key: position_state
          value: '0.000000'
      canopen_states: []
      ethercat_states: []
      vda5050_states: []

.. note::

   The custom diagnostics node and its timer are created only if the executor is successfully passed to the hardware component. If you don't see the topic or node, ensure the hardware plugin is correctly implemented and that the controller manager is providing an executor.
.. _hardware_status_publisher_implementation:

Implementation Details of the Hardware Status Publisher
-------------------------------------------------------
**1. Using the Framework-Managed Status Publisher (Recommended for HardwareStatus Messages)**

   The ``ros2_control`` framework provides a built-in, real-time safe mechanism for publishing standardized hardware status via the ``control_msgs/msg/HardwareStatus`` message. This is the simplest and most robust way to provide detailed status messages. It is enabled by implementing two virtual methods in your hardware component.

   a. **Override configure_hardware_status_message**: This non-realtime method is called once during initialization. You must override it to define the **static structure** of your status message. This includes setting the ``hardware_id``, resizing the ``hardware_device_states`` vector, and for each device, resizing its specific status vectors (e.g., ``generic_hardware_status``) and populating static fields like ``device_id``.

      .. code-block:: cpp

         // In rrbot.hpp, add the override declaration:
         hardware_interface::CallbackReturn configure_hardware_status_message(
           control_msgs::msg::HardwareStatus & msg_template) override;

         // In rrbot.cpp
         hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::configure_hardware_status_message(
           control_msgs::msg::HardwareStatus & msg)
         {
           msg.hardware_id = get_hardware_info().name;
           msg.hardware_device_states.resize(get_hardware_info().joints.size());

           for (size_t i = 0; i < get_hardware_info().joints.size(); ++i)
           {
             msg.hardware_device_states[i].device_id = get_hardware_info().joints[i].name;
             // This example uses one generic status per joint
             msg.hardware_device_states[i].generic_hardware_status.resize(1);
           }
           return hardware_interface::CallbackReturn::SUCCESS;
         }

   b. **Override update_hardware_status_message**: This real-time safe method is called from the framework's internal timer. You override it to **fill in the dynamic values** of the pre-structured message, typically by copying your hardware's internal state variables (updated in your ``read()`` method) into the fields of the message.

      .. code-block:: cpp

         // In rrbot.hpp, add the override declaration:
         hardware_interface::return_type update_hardware_status_message(
           control_msgs::msg::HardwareStatus & msg) override;

         // In rrbot.cpp
         hardware_interface::return_type RRBotSystemPositionOnlyHardware::update_hardware_status_message(
           control_msgs::msg::HardwareStatus & msg)
         {
           for (size_t i = 0; i < get_hardware_info().joints.size(); ++i)
           {
             auto & generic_status = msg.hardware_device_states[i].generic_hardware_status;
             // Example: Map internal state to a standard status field
             double position = get_state(get_hardware_info().joints[i].name + "/position");
             if (std::abs(position) > 2.5) // Arbitrary warning threshold
             {
               generic_status.health_status = control_msgs::msg::GenericState::HEALTH_WARNING;
             }
             else
             {
               generic_status.health_status = control_msgs::msg::GenericState::HEALTH_OK;
             }
             generic_status.operational_mode = control_msgs::msg::GenericState::MODE_AUTO;
             generic_status.power_state = control_msgs::msg::GenericState::POWER_ON;
           }
           return hardware_interface::return_type::OK;
         }

   c. **Enable in URDF**: To activate the publisher, add the ``status_publish_rate`` parameter to your ``<hardware>`` tag in the URDF. Setting it to 0.0 disabled the feature.

      .. code-block:: xml

         <ros2_control name="RRBotSystemPositionOnly" type="system">
           <hardware>
             <plugin>ros2_control_demo_example_17/RRBotSystemPositionOnlyHardware</plugin>
             <param name="status_publish_rate">20.0</param> <!-- Defaults to 0.0 Hz -->
           </hardware>
           ...
         </ros2_control>

      This will create a publisher on the topic ``/rrbot/hardware_status``.

.. _diagnostic_publisher_implementation:

Implementation Details of the Diagnostic Publisher
--------------------------------------------------

This example demonstrates the two recommended ways for a hardware component to perform ROS 2 communications: using the standard ``diagnostic_updater`` on the default node, and creating a separate custom node added to the Controller Manager's executor.

**1. Using the ``diagnostic_updater`` with the Default Node (Standard Method)**

The ``diagnostic_updater`` library is the standard ROS 2 tool for publishing diagnostics. It automatically handles timer creation and publishing to the ``/diagnostics`` topic, making it the preferred method. The ``HardwareComponentInterface`` provides a ``get_node()`` method to access the default node, which is passed to the updater.

The key steps are:
1.  **Create an Updater**: Instantiate ``diagnostic_updater::Updater``, passing it the default node. The updater internally creates a publisher to ``/diagnostics`` and a periodic timer (default 1 Hz).
2.  **Set Hardware ID**: Set a unique identifier for the hardware component.
3.  **Add a Diagnostic Task**: Add a function that will be called periodically by the updater's internal timer. This function populates a ``DiagnosticStatusWrapper`` with the current status of the hardware.

.. code-block:: cpp

  #include "diagnostic_updater/diagnostic_updater.hpp"

  if (get_node())
  {
    updater_ = std::make_shared<diagnostic_updater::Updater>(get_node());
    updater_->setHardwareID(get_hardware_info().name);

    updater_->add(
      get_hardware_info().name + " Status", this, &RRBotSystemPositionOnlyHardware::produce_diagnostics);
  }

  void RRBotSystemPositionOnlyHardware::produce_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    // Add status summary
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Hardware is OK");
    // Optionally add key-value pairs
    // stat.add("voltage", "24.1V");
  }


**2. Creating a Custom Node with the Executor (Advanced Method)**

For non-diagnostic topics or when a separate node identity is required, a hardware component can create its own node and add it to the Controller Manager's executor.

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
      custom_status_node_ = std::make_shared<rclcpp::Node>(get_hardware_info().name + "_custom_node");
      locked_executor->add_node(custom_status_node_->get_node_base_interface());


4.  **Publishing Status Messages**: A publisher and a periodic wall timer are created on the new custom node. The timer's callback periodically publishes a status message, demonstrating that the node is alive and running alongside the main control loop.

.. code-block:: cpp

      // Inside the `if (auto locked_executor = ...)` block
      custom_status_publisher_ =
        custom_status_node_->create_publisher<std_msgs::msg::String>("rrbot_custom_status", 10);

      custom_status_timer_ = custom_status_node_->create_wall_timer(
        2s, [this](){ /* ... lambda to publish message ... */ });

**3. (Extra)Using the Default Node, but with a Custom Publisher**

This is not implemented in the example, but is also a viable option.

.. code-block:: cpp

    // Get Default Node added to executor
    auto default_node = get_node();
    if (default_node)
    {
        default_status_publisher_ = default_node->create_publisher<std_msgs::msg::String>("rrbot_default_status", 10);
        using namespace std::chrono_literals;
        default_status_timer_ = default_node->create_wall_timer(2.5s, [this](){ /* ... */ });
    }


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

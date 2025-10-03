# ROS2 Control - Example 1 (RRBot System Position Only)

This example demonstrates the **basic architecture of ROS2 Control** using a simple 2-joint robot (**RRBot**) with a custom hardware interface and controllers.

---

## Hardware

The robot (**RRBot**) is a simple 2-joint robot.

A custom hardware interface **`RRBotSystemPositionOnlyHardware`** is implemented in:  
`ros2_control_demo_example_1/rrbot_hardware_interface.cpp`

This hardware plugin:

- Inherits from **`hardware_interface::SystemInterface`**
- Defines how to **read joint positions** (from hardware, simulated here)
- Defines how to **write commands** (set joint positions)
- Is registered in a **`plugin.xml`** file so the **ResourceManager** can load it

From the UML:  
- `hardware_interface::SystemInterface` → implemented by the RRBot hardware plugin  
- `hardware_interface::ResourceManager` → manages your hardware plugin after loading it  

---

## Robot Description (URDF/Xacro)

The robot description is defined in:  
`rrbot.urdf.xacro`

The URDF contains:

- Links and joints of the robot  
- A `<ros2_control>` tag specifying which hardware plugin to load  

When launched, **ros2_control** reads this URDF and knows:

- Which joints exist  
- Which hardware plugin provides those joints  

From the UML:  
- `robot_config (URDF)` → parsed into **`hardware_interface::HardwareInfo`** and passed to **ResourceManager**  

---

## Controller Manager

The **`controller_manager::ControllerManager`** is the core ROS2 node.  

Responsibilities:

- Loads your hardware plugin (via **ResourceManager**)  
- Loads and manages controllers (defined in a YAML config file)  

From the UML:  
- `controller_manager::ControllerManager` → central node  
- Uses `controllers_config (YAML)` and `robot_config (URDF)` to configure everything  

---

## Controllers

**Forward Command Controller**  
- `forward_position_controller` (from `forward_command_controller` package)  
- Directly forwards position commands to the hardware interface  

**Joint State Broadcaster**  
- Publishes **`/joint_states`**  
- Allows visualization of the robot in RViz  

From the UML:  
- `controller_interface::ControllerInterface` → base class for controllers  
- `forward_command_controller::ForwardCommandController` → used in this demo  
- `joint_state_broadcaster::JointStateBroadcaster` → publishes joint states  

---


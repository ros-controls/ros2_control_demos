# ROS 2 Control for a 6 DOF Robot
ROS 2 control is a realtime control framework designed for general robotics applications. Standard c++ interfaces exist for interacting with hardware and querying user defined controller commands. These interfaces enhance code modularity and robot agnostic design. Application specific details, e.g. what controller to use, how many joints a robot has and their kinematic structure, are specified via YAML parameter configuration files and a Universal Robot Description File (URDF). Finally, the ROS 2 control framework is deployed via ROS 2 launch a file.        

## ROS 2 control architecture

This tutorial will address each component of ROS 2 control in detail, namely: 
1. Writing a URDF
1. Writing a hardware interface
1. Writing a controller
1. Writing a parameter configuration file
1. Writing a ROS 2 control launch file

## Writing a URDF
The URDF file is a standard XML based file used to describe characteristic of a robot. For ROS 2 control, there are three primary tags: `link`, `joint`, and `ros2_control`. The `joint` tag define the robot's kinematic structure, while the `link` tag defines the dynamic properties and 3D geometry. The `ros2_control` defines the hardware and controller configuration. 
### Geometry
Most commercial robots already have `robot_description` packages defined, see the [Universal Robots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) for an example. However, this tutorial will go through the details of creating one from scratch. 

First, we need a 3D model of our robot. For illustration, a generic 6 DOF robot manipulator will be used. 
<p align="center">
  <img src="resources/robot.png" width="350" title="hover text">
</p>

The robot's 6 links each need to be processed and exported to their own `.stl` and `.dae` files. Generally, the `.stl` 3D model files are coarse meshes used for fast collision checking, while the `.dae` files are used for visualization purposed only. We will use the same mesh in our case for simplicity.   

By convention, each `.stl` file expresses the position its vertices in its own reference frame. Hence, we need to specify the linear transformation (rotation and translation) between each link to define the robot's full geometry. The 3D model for each link should be adjusted such that the proximal joint axis (the axis that connects the link to its parent) is in the z-axis direction. The 3D model's origin should also be adjusted such that the bottom face of the mesh is co-planer with the xy-plane. Thw following mesh illustrates this configuration.    
<p align="center">
  <img src="resources/link_1.png" width="200" title="hover text">
</p>
<p align="center">
  <img src="resources/link_2_aligned.png" width="350" title="hover text">
</p>

Each mesh should be exported to its own file after processing them. [Blender](https://www.blender.org/) is an open source 3D modeling software, which can import/export `.stl` and `.dae` files and manipulate their vertices. Blender was used to process the robot model in this tutorial. 

We can finally calculate the transforms between the robot's joints and begin writing the URDF. First, apply a negative 90 degree roll to link 2 in its frame.    

<p align="center">
  <img src="resources/link_2_roll.png" width="350" title="hover text">
</p>

To keep the example simple, we will not apply a pitch now. Then, we apply a positive 90 degree yaw.

<p align="center">
  <img src="resources/link_2_roll_yaw.png" width="350" title="hover text">
</p>

Finally, we apply a translation of -0.1 meters in the x-axis and 0.18 meters in the z-axis between the link 2 and link 1 frame. The final result is shown below.

<p align="center">
  <img src="resources/link_2_roll_yaw_trans.png" width="350" title="hover text">
</p>

The described process is then repeated for all links.  

### URDF file
The URDF file is generally formatted according to the following template. 
```xml
<robot name="robot_6_dof">
  <!-- create link fixed to the "world" -->
  <link name="base_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_6_dof/meshes/visual/link_0.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_6_dof/meshes/collision/link_0.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <!-- additional links ... -->
  <link name="world"/>
  <link name="tool0"/>
  <joint name="base_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <!-- joints - main serial chain -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin rpy="0 0 0" xyz="0 0 0.061584"/>
    <axis xyz="0 0 1"/>
    <limit effort="1000.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="2.5"/>
  </joint>
  <!-- additional joints ... -->
  <!-- ros2 control tag -->
  <ros2_control name="robot_6_dof" type="system">
    <hardware>
      <plugin>
        <!-- {Name_Space}/{Class_Name}-->
      </plugin>
    </hardware>
    <joint name="joint_1">
      <command_interface name="position">
        <param name="min">{-2*pi}</param>
        <param name="max">{2*pi}</param>
      </command_interface>
      <!-- additional command interfaces ... -->
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <!-- additional state interfaces ... -->
    </joint>
    <!-- additional joints ...-->
    <!-- additional hardware/sensors ...-->
  </ros2_control>
</robot>
```
* The `robot` tag encloses all contents of the URDF file. It has a name attribute which must be specified. 
* The `link` tag defines the robot's geometry and inertia properties. It has a name attribute which will be referred to by the `joint` tags.
* The `visual` tag specifies the rotation and translation of the visual mesh. If the meshes were process as described previously, then the `orgin` tag can be left at all zeros.
* The `geometry` and `mesh` tags specify the location of the 3D mesh file relative to a specified ROS 2 package.
* The `collision` tag is equivalent to the `visual` tag, except the specified mesh is used for commission checking in some applications. 
* The `inertial` tag specifies mass and inertia for the link. The origin tag specifies the link's center of mass. These values are used to calculate forward and inverse dynamics. Since our application does not use dynamics, uniform arbitrary values are used.
* The `<!-- additional links ... -->` comments indicates that many consecutive `link` tags will be defined, one for each link.
* The `<link name="world"/>` and `<link name="tool0"/>` elements are not required. However, it is convention to set the link at the tip of the robot to  tool0 and to define the robot's base link relative to a world frame.
* The `joint` tag specifies the kinematic structure of the robot. It two required attributes: name and type. The type specifies the viable motion between the two connected links. The subsequent `parent` and `child` links specify which two links are joined by the joint.
* The `axis` tag species the joint's axis of rotation. If the meshes were process as described previously, then the axis value is always `"0 0 1"`.
* The `limits` tag specifies kinematic and dynamics limits fo the joint. 
* The `ros2_control` tag specifies 

Explain `ros2_control` concepts


urdf_to_graphviz pr2.urdf


## Writing a hardware interface


## Writing a controller


## Writing a reference generator
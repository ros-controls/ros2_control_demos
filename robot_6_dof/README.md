# ROS 2 Control for a 6 DOF Robot
ROS 2 control is a realtime control framework designed for general robotics applications. Standard c++ interfaces exist for interacting with hardware and querying user defined controller commands. These interfaces enhance code modularity and robot agnostic design. Application specific details, e.g. what controller to use, how many joints a robot has and their kinematic structure, are specified via YAML parameter configuration files and a Universal Robot Description File (URDF). Finally, the ROS 2 control framework is deployed via ROS 2 launch a file.        

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
  <img src="resources/link_2.png" width="350" title="hover text">
</p>

Each mesh should be exported to its own file after processing them. [Blender](https://www.blender.org/) is an open source 3D modeling software, which can import/export `.stl` and `.dae` files and manipulate their vertices. Blender was used to process the robot model in this tutorial.     



Explain `ros2_control` concepts


urdf_to_graphviz pr2.urdf


## Writing a hardware interface


## Writing a controller


## Writing a reference generator
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            # gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathSubstitution(FindPackageShare("ros_gz_sim"))
                        / "launch"
                        / "gz_sim.launch.py"
                    ]
                ),
                launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathSubstitution(FindPackageShare("ros_gz_sim"))
                        / "launch"
                        / "gz_sim.launch.py"
                    ]
                ),
                launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
                condition=UnlessCondition(LaunchConfiguration("gui")),
            ),
            # Gazebo bridge
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "rrbot_system_position",
                    "-allow_renaming",
                    "true",
                ],
            ),
            # robot_state_publisher with robot_description from xacro
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_9"))
                                / "urdf"
                                / "rrbot.urdf.xacro",
                                " ",
                                "use_gazebo:=true",
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "forward_position_controller",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_9"))
                    / "config"
                    / "rrbot_controllers.yaml",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("ros2_control_demo_description"))
                    / "rrbot/rviz"
                    / "rrbot.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )

# Copyright 2020 ros2_control Development Team
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
            DeclareLaunchArgument(
                "remap_odometry_tf",
                default_value="false",
                description="Remap odometry TF from the steering controller to the TF tree.",
            ),
            # Control node
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_11"))
                    / "config"
                    / "carlikebot_controllers.yaml"
                ],
                output="both",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_11"))
                                / "urdf"
                                / "carlikebot.urdf.xacro",
                            ]
                        )
                    }
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
                    / "carlikebot/rviz"
                    / "carlikebot.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            # the steering controller libraries by default publish odometry on a separate topic than /tf
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "bicycle_steering_controller",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_11"))
                    / "config"
                    / "carlikebot_controllers.yaml",
                    "--controller-ros-args",
                    "-r /bicycle_steering_controller/tf_odometry:=/tf",
                ],
                condition=IfCondition(LaunchConfiguration("remap_odometry_tf")),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "bicycle_steering_controller",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_11"))
                    / "config"
                    / "carlikebot_controllers.yaml",
                ],
                condition=UnlessCondition(LaunchConfiguration("remap_odometry_tf")),
            ),
        ]
    )

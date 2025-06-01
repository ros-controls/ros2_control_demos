# Copyright 2025 ros2_control Development Team
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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "python3",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros2_control_demo_example_17"),
                            "launch",
                            "demo_test_helper.py",
                        ]
                    ),
                ],
                output="screen",
            )
        ]
    )

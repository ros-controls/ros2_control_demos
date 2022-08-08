# Copyright 2021 Department of Engineering Cybernetics, NTNU.
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rrbot_base.launch.py"]),
        launch_arguments={
            "controllers_file": "rrbot_jtc_ruckig_one_dof_test.yaml",
            "description_file": "rrbot_system_multi_interface.urdf.xacro",
            "prefix": "",
            "use_fake_hardware": "true",
            "fake_sensor_commands": "false",
            "slowdown": "1.0",
            "robot_controller": "joint_trajectory_controller_ruckig_smoothing",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [base_launch])

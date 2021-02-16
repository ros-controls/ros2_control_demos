# Copyright (c) 2021 PAL Robotics S.L. All rights reserved.
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

from controller_manager.launch_utils import generate_load_controller_launch_description


def generate_launch_description():
    return generate_load_controller_launch_description(
        controller_name='forward_command_controller_position',
        controller_type=None,
        controller_params_file=None)

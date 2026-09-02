# Copyright (C) 2026 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Julia Jia

import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    description_package = "ros2_control_demo_description"
    description_file = "open_duck_mini.urdf.xacro"

    description_file_path = os.path.join(
        get_package_share_directory(description_package),
        "openduckmini",
        "urdf",
        description_file,
    )

    _, tmp_urdf_output_file = tempfile.mkstemp(suffix=".urdf")

    xacro_command = (
        f"{shutil.which('xacro')}"
        f" {description_file_path}"
        f" headless:=true"
        f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert (
            check_urdf_process.returncode == 0
        ), "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description. Please check your xacro file."

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == "__main__":
    test_urdf_xacro()

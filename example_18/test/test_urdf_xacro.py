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
import xml.etree.ElementTree as ET

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


def test_ros2_control_joints_exist_in_urdf():
    """Joint names in the ros2_control tag must refer to URDF/MuJoCo joints, also when args are passed."""
    description_file_path = os.path.join(
        get_package_share_directory("ros2_control_demo_description"),
        "openduckmini",
        "urdf",
        "open_duck_mini.urdf.xacro",
    )
    # prefix is not supported (the URDF and MuJoCo model have fixed names) and must be a no-op
    xacro_process = subprocess.run(
        [shutil.which("xacro"), description_file_path, "headless:=true", "prefix:=duck_"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert xacro_process.returncode == 0, xacro_process.stderr

    robot = ET.fromstring(xacro_process.stdout)
    urdf_joints = {joint.get("name") for joint in robot.findall("joint")}
    control_joints = [joint.get("name") for joint in robot.find("ros2_control").findall("joint")]
    assert len(control_joints) == 14
    missing = [name for name in control_joints if name not in urdf_joints]
    assert not missing, f"ros2_control joints not in URDF: {missing}"


def test_urdf_joint_origins_match_mjcf():
    """robot_state_publisher must show the same kinematics as the simulated MuJoCo model."""
    mujoco_dir = os.path.join(
        get_package_share_directory("ros2_control_demo_description"), "openduckmini", "mujoco"
    )
    urdf = ET.parse(os.path.join(mujoco_dir, "robot.urdf")).getroot()
    mjcf = ET.parse(os.path.join(mujoco_dir, "open_duck_mini_v2.xml")).getroot()

    # MJCF: the body that owns a hinge joint is placed relative to its parent like the URDF joint
    mjcf_body_pos = {}
    for body in mjcf.iter("body"):
        for joint in body.findall("joint"):
            mjcf_body_pos[joint.get("name")] = [float(v) for v in body.get("pos").split()]

    checked = 0
    for joint in urdf.iter("joint"):
        name = joint.get("name")
        if name not in mjcf_body_pos:
            continue
        xyz = [float(v) for v in joint.find("origin").get("xyz").split()]
        for axis, (u, m) in enumerate(zip(xyz, mjcf_body_pos[name])):
            assert abs(u - m) < 1e-4, f"{name} origin[{axis}]: urdf={u} mjcf={m}"
        checked += 1
    assert checked == 14, f"expected 14 actuated joints, compared {checked}"


if __name__ == "__main__":
    test_urdf_xacro()
    test_urdf_joint_origins_match_mjcf()

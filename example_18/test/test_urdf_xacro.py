import os
import shutil
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory

def test_urdf_xacro():
    path = os.path.join(get_package_share_directory("ros2_control_demo_example_18"), "urdf", "diffbot.urdf.xacro")
    fd, output = tempfile.mkstemp(suffix=".urdf"); os.close(fd)
    try:
        assert subprocess.run([shutil.which("xacro"), path], stdout=open(output, "w"), check=False).returncode == 0
        assert subprocess.run([shutil.which("check_urdf"), output], capture_output=True).returncode == 0
    finally:
        os.remove(output)

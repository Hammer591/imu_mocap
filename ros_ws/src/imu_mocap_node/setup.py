from setuptools import setup
from glob import glob
import os
import stat

package_name = "imu_mocap_node"

# Install scripts to lib/{package_name}/ for ros2 run (setuptools 72.x workaround)
_script_src = sorted(glob("scripts/*"))
_script_install_dir = os.path.join("lib", package_name)

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/mocap.launch.py"]),
        (_script_install_dir, _script_src),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hammer",
    maintainer_email="hammer@example.com",
    description="ROS2 node for STM32 V2/V3 imu-mocap protocol",
    license="MIT",
    tests_require=["pytest"],
)

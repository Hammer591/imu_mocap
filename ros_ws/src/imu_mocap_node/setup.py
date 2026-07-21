from setuptools import setup

package_name = "imu_mocap_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/mocap.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hammer",
    maintainer_email="hammer@example.com",
    description="ROS2 node for STM32 V2/V3 imu-mocap protocol",
    license="MIT",
    tests_require=["pytest"],
)

from setuptools import setup, find_packages

package_name = "xarm_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/bridge_params.yaml",
            "config/rosbridge_allowlist.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/bridge.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan Cunningham",
    maintainer_email="Ryan-Cunningham_EZAG@users.noreply.github.com",
    description="FastAPI REST bridge and rosbridge launch for xArm web control",
    license="MIT",
    url="https://github.com/xArm-Developer/xarm_ros2",
    entry_points={
        "console_scripts": [
            "bridge_node = xarm_bridge.bridge_node:main",
        ],
    },
)

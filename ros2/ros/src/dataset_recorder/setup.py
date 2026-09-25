from setuptools import find_packages, setup

package_name = "dataset_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/recorder.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Service-controlled, automatically segmented ROS 2 dataset recorder.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "recorder_node = dataset_recorder.recorder_node:main",
        ],
    },
)

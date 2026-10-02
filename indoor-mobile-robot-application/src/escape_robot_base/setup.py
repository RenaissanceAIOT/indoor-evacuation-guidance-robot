from setuptools import find_packages, setup

package_name = "escape_robot_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="RenaissanceAIOT",
    maintainer_email="166830931+RenaissanceAIOT@users.noreply.github.com",
    description="Laboratory omnidirectional base driver for ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_driver = escape_robot_base.base_driver_node:main",
            "mock_base = escape_robot_base.mock_base_node:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "escape_robot_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RenaissanceAIOT",
    maintainer_email="166830931+RenaissanceAIOT@users.noreply.github.com",
    description="RGB-D and OpenCV perception nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "object_depth = escape_robot_perception.object_depth_node:main",
            "visual_behavior = escape_robot_perception.visual_behavior_node:main",
            "gesture = escape_robot_perception.gesture_node:main",
        ],
    },
)


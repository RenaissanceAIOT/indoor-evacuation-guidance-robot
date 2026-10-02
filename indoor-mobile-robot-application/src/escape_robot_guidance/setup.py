from setuptools import find_packages, setup

package_name = "escape_robot_guidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RenaissanceAIOT",
    maintainer_email="166830931+RenaissanceAIOT@users.noreply.github.com",
    description="Evacuation guidance coordinator",
    license="MIT",
    entry_points={
        "console_scripts": [
            "evacuation_coordinator = escape_robot_guidance.evacuation_coordinator:main",
            "system_supervisor = escape_robot_guidance.system_supervisor:main",
        ],
    },
)

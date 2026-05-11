from setuptools import setup
from glob import glob
import os

package_name = "boat_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Baseline controller for surface boat navigation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "boat_controller = boat_control.boat_controller_node:main",
            "waypoint_publisher = boat_control.waypoint_publisher_node:main",
            "disturbance_generator = boat_control.disturbance_generator_node:main",
            "disturbance_applier = boat_control.disturbance_applier_node:main",
            "metrics_logger = boat_control.metrics_logger_node:main",
            "metrics_report = boat_control.metrics_report:main",
        ],
    },
)

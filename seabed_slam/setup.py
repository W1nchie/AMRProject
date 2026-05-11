from glob import glob
import os

from setuptools import setup


package_name = "seabed_slam"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "data"), glob("data/missouri_i64/*")),
        # (os.path.join("share", package_name, "data"), glob("data/")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Seabed mapping nodes and Gazebo worlds for the AMR project.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sonar_mapper = seabed_slam.sonar_mapper_node:main",
            "map_evaluator = seabed_slam.map_evaluator_node:main",
            "seabed_mcl = seabed_slam.seabed_mcl_node:main",
            "forward_motion = seabed_slam.forward_motion_node:main",
            "reference_map_recorder = seabed_slam.reference_map_recorder_node:main",
        ],
    },
)

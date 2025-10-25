from setuptools import setup, find_packages

package_name = "robot_3r_planners"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=[f"{package_name}", f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Clinton Enwerem",
    maintainer_email="me@clintonenwerem.com",
    description="Motion planning ROS 2 suite for a spatial 3R kinematic chain.",
    license="BSD",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sampling_based_planner = robot_3r_planners.scripts.sampling_based_planner:main",
            "simple_obstacle_publisher = robot_3r_planners.scripts.simple_obstacle_publisher:main",
            "robot_geom_publisher = robot_3r_planners.scripts.robot_geom_publisher:main",
        ],
    },
)

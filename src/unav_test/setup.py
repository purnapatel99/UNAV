from setuptools import find_packages, setup

package_name = "unav_test"

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
    maintainer="purna",
    maintainer_email="purnapatel99@gmail.com",
    description="Test nodes for the autonomous drone system.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "test_mission = unav_test.test_mission_node:main",
            "straight_test = unav_test.straight_test_node:main",
            "follow_point_test = unav_test.follow_point_test_node:main",
            "object_simulator = unav_test.object_simulator_node:main",
            "follow_object_test = unav_test.follow_object_test_node:main",
            "f1_replay = unav_test.f1_replay_node:main",
        ],
    },
)

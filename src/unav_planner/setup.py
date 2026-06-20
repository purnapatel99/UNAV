from setuptools import find_packages, setup

package_name = "unav_planner"

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
    description="Orchestrator node and trajectory generator library for autonomous drone.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "orchestrator = unav_planner.orchestrator_node:main",
        ],
    },
)

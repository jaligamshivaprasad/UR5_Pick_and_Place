from setuptools import find_packages, setup


package_name = "ur5_data_logger"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Read-only experiment logger for UR5 controller tracking data.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "experiment_logger = ur5_data_logger.logger:main",
        ],
    },
)

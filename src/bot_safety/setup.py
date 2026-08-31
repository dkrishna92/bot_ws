from setuptools import find_packages, setup

package_name = "bot_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Krishna",
    maintainer_email="krishna.dontaraju@gmail.com",
    description="Secondary, defense-in-depth ROS 2 watchdog node (not the primary MCU-based e-stop)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "watchdog_node = bot_safety.watchdog_node:main",
        ],
    },
)

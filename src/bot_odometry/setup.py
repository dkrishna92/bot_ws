from setuptools import find_packages, setup

package_name = "bot_odometry"

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
    description="Wheel odometry + ultrasonic from Teensy-reported serial data (real hardware only)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wheel_odom_node = bot_odometry.wheel_odom_node:main",
        ],
    },
)

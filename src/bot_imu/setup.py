from setuptools import find_packages, setup

package_name = "bot_imu"

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
    description="Bosch BNO055 IMU driver node (I2C, NDOF fusion mode)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bno055_node = bot_imu.bno055_node:main",
        ],
    },
)

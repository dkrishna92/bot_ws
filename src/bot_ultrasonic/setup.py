from setuptools import find_packages, setup

package_name = "bot_ultrasonic"

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
    description="Ultrasonic sensor array node (HC-SR04-class, bit-banged GPIO)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ultrasonic_node = bot_ultrasonic.ultrasonic_node:main",
        ],
    },
)

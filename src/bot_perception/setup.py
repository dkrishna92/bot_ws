from setuptools import find_packages, setup

package_name = "bot_perception"

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
    description="Vision-based autonomous start signal detector",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "start_trigger_node = bot_perception.start_trigger_node:main",
            "sensor_fusion_node = bot_perception.sensor_fusion_node:main",
        ],
    },
)

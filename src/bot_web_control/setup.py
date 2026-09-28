from setuptools import find_packages, setup
import os
from glob import glob

package_name = "bot_web_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "static"), glob("static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Krishna",
    maintainer_email="krishna.dontaraju@gmail.com",
    description="Localhost web dashboard for start/stop/mapping-run control, live map view, and WASD teleop",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_control_node = bot_web_control.web_control_node:main",
        ],
    },
)

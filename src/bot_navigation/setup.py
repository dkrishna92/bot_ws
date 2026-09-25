from setuptools import find_packages, setup

package_name = "bot_navigation"

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
    description="Multi-lap course/race logic: sends Nav2 the checkpoint route for the current course once the vision start signal fires.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lap_navigator_node = bot_navigation.lap_navigator_node:main",
        ],
    },
)

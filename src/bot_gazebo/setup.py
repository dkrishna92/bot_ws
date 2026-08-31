from setuptools import find_packages, setup

package_name = 'bot_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/bot.urdf.xacro']),
        ('share/' + package_name + '/meshes', ['meshes/base_link.dae']),
        ('share/' + package_name + '/worlds', [
            'worlds/obstacle_course.world',
            'worlds/speed_course.world',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/gazebo.launch.py',
            'launch/gazebo_sim.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Krishna',
    maintainer_email='krishna.dontaraju@gmail.com',
    description='Gazebo simulation for the DIY Robot Challenge vehicle',
    license='Apache-2.0',
)

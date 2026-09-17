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
            'worlds/obstacle_course_cfr.world',
            'worlds/speed_course_cfr.world',
            'worlds/circular_course.world',
            'worlds/onshape_course.world',
        ]),
        ('share/' + package_name + '/models/cfr_arduino_bridge', [
            'models/cfr_arduino_bridge/model.config',
            'models/cfr_arduino_bridge/model.sdf',
        ]),
        ('share/' + package_name + '/models/cfr_arduino_bridge/meshes', [
            'models/cfr_arduino_bridge/meshes/bank.stl',
            'models/cfr_arduino_bridge/meshes/bucket.stl',
            'models/cfr_arduino_bridge/meshes/car_wash_arch.stl',
            'models/cfr_arduino_bridge/meshes/car_wash_base.stl',
            'models/cfr_arduino_bridge/meshes/gravel_box.stl',
            'models/cfr_arduino_bridge/meshes/hoop.stl',
            'models/cfr_arduino_bridge/meshes/pothole_board.stl',
            'models/cfr_arduino_bridge/meshes/pothole_bump.stl',
            'models/cfr_arduino_bridge/meshes/ramp_narrowing.stl',
            'models/cfr_arduino_bridge/meshes/ramp_pothole_entry.stl',
            'models/cfr_arduino_bridge/meshes/ramp_up.stl',
            'models/cfr_arduino_bridge/meshes/start_signal_arm_green.stl',
            'models/cfr_arduino_bridge/meshes/start_signal_arm_red.stl',
            'models/cfr_arduino_bridge/meshes/start_signal_frame.stl',
            'models/cfr_arduino_bridge/meshes/tunnel.stl',
        ]),
        ('share/' + package_name + '/models/onshape_course/meshes', [
            'models/onshape_course/meshes/course_layout.stl',
        ]),
        ('share/' + package_name + '/models/onshape_course', [
            'models/onshape_course/model.config',
            'models/onshape_course/model.sdf',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/gazebo.launch.py',
            'launch/gazebo_sim.launch.py',
            'launch/obstacle_course_cfr.launch.py',
            'launch/speed_course_cfr.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Krishna',
    maintainer_email='krishna.dontaraju@gmail.com',
    description='Gazebo simulation for the DIY Robot Challenge vehicle',
    license='Apache-2.0',
)

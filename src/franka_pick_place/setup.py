from setuptools import find_packages, setup

package_name = 'franka_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pick_place.launch.py', 'launch/conveyor_control.launch.py']),
        ('share/' + package_name + '/config', ['config/fr3_moveit_joint_limits.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chahed',
    maintainer_email='moahmedali.chahed69@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place_node = franka_pick_place.pick_place_node:main',
            'conveyor_controller_node = franka_pick_place.conveyor_controller_node:main',
            'cube_detector_node = franka_pick_place.cube_detector_node:main',
        ],
    },
)

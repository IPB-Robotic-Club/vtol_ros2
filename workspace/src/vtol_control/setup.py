from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vtol_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pilot',
    maintainer_email='pilot@vtol-dev.local',
    description='Package for autonomous and manual control of VTOL drone using MAVROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vtol_core = vtol_control.vtol_core:main',
            'vehicle_status = vtol_control.vehicle_status:main',
            'test_arm = vtol_control.test_arm:main',
            'menu_launcher = vtol_control.menu_launcher:main',
            'mission_hover = vtol_control.mission_hover:main',
            'mission_maneuver = vtol_control.mission_maneuver:main',
            'mission_centering = vtol_control.mission_centering:main',
            'servo_drop = vtol_control.servo_drop:main',
        ],
    },
)

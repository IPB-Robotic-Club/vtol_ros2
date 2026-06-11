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
            'keyboard_teleop = vtol_control.keyboard_teleop:main',
            'test_flight = vtol_control.test_flight:main',
        ],
    },
)

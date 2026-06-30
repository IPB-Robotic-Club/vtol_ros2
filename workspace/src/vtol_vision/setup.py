from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vtol_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pilot',
    maintainer_email='pilot@todo.todo',
    description='Package for VTOL vision processing, including UDP receiver and ArUco detection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_receiver = vtol_vision.aruco_receiver:main',
        ],
    },
)

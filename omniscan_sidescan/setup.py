from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'omniscan_sidescan'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # Keep the upstream ping-python licence alongside the vendored code.
    package_data={'omniscan_sidescan.brping': ['LICENSE']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Monika Roznere',
    maintainer_email='mrozner1@binghamton.edu',
    description='ROS 2 driver for the Cerulean Omniscan 450 side scan sonar.',
    license='MIT',
    # colcon picks the pytest runner over `setup.py test` based on this, so it
    # has to stay even though setuptools reports it as deprecated.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sidescansonar_node = omniscan_sidescan.sidescansonar_node:main',
            'sidescansonarall_node = omniscan_sidescan.sidescansonarall_node:main',
            'omniscan450_simulator = omniscan_sidescan.omniscan450_simulator:main',
        ],
    },
)

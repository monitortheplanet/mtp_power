# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: ament_python build description for this package.

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mtp_power'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alessandro Berruti',
    maintainer_email='berruti.alessandro@gmail.com',
    description=(
        'Power the robot off from a held radio button, refusing unless the '
        'wheel odometry proves it is standing still, and say so on '
        '/power/state so the buzzer and the LED can warn a human.'
    ),
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'power_manager = mtp_power.power_manager_node:main',
        ],
    },
)

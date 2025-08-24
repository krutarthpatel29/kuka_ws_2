# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

from setuptools import setup

package_name = 'kuka_kr360_middleware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krutarth',
    maintainer_email='st184775@stud.uni-stuttgart.de',
    description='OPC UA bridge for KUKA KR360 control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opcua_kr360_bridge = kuka_kr360_middleware.opcua_kr360_bridge:main',
        ],
    },
)

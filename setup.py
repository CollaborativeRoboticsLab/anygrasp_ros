from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'anygrasp_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalana',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'anygrasp_detection_node = anygrasp_ros.anygrasp_detection_node:main',
            'anygrasp_tracking_node = anygrasp_ros.anygrasp_tracking_node:main',
        ],
    },
)

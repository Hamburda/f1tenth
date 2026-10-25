import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rares',
    maintainer_email='rares.hamburda@gmail.com',
    description='Pure pursuit path tracking (Lab 5)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main'
        ],
    },
)

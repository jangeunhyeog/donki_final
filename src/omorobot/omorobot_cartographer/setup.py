import os # add
from glob import glob # add
from setuptools import find_packages, setup

package_name = 'omorobot_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')), # add
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # add
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')), # add
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr.K',
    maintainer_email='t.shaped.person@gmail.com',
    description='Launch scripts for cartographer',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

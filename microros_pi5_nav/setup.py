import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'microros_pi5_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Saved maps travel with the package so `map:=` can take a bare name.
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.yaml') + glob('maps/*.pgm') + glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pacharawit',
    maintainer_email='akeeslatun@gmail.com',
    description='SLAM and Nav2 configuration for the Yahboom MicroROS-Pi5 4WD car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_qos_relay = microros_pi5_nav.scan_qos_relay:main',
        ],
    },
)

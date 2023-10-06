from setuptools import setup
import os
from glob import glob

package_name = 'simple_commander_for_foxy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuta',
    maintainer_email='yutanakamura01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_commander = simple_commander_for_foxy.simple_commander:main',
            'waypoint_saver = simple_commander_for_foxy.waypoint_saver:main'
        ],
    },
)

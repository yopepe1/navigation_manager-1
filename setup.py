from setuptools import setup
import os
from glob import glob

package_name = 'navigation_manager'

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
            'navigation_manager = navigation_manager.navigation_manager:main',
            'waypoint_saver = navigation_manager.waypoint_saver:main',
            'param_test = navigation_manager.param_test:main'
        ],
    },
)

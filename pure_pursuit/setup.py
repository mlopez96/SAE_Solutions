from setuptools import setup
import os
from glob import glob


package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huzefashk',
    maintainer_email='hkagalw@g.clemson.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit=pure_pursuit.pure_pursuit:main',
            'waypoint_logger=pure_pursuit.waypoint_logger:main',
        ],
    },
)

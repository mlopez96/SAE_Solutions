from setuptools import setup
import os
from glob import glob

package_name = 'capstone_project'

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
    maintainer='rushabhtalati',
    maintainer_email='rushabh806@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'line_follower=capstone_project.line_follower:main',# Insert executable entry point here
        'wall_follower=capstone_project.wall_follower:main',# Insert executable entry point here
        'classify_resnet50=capstone_project.classify_ResNet50:main',# Insert executable entry point here
        'state_machine=capstone_project.state_machine:main',# Insert executable entry point here
        #'pure_pursuit=capstone_project.pure_pursuit:main',# Insert executable entry point here
        'waypoint_logger=capstone_project.waypoint_logger:main',# Insert executable entry point here
        'tiny_yolov3_lite=capstone_project.classify_tinyyolov3_lite:main',
        'go_to_waypoint=capstone_project.go_to_waypoint:main', #Insert Executable Here
        ],
    },
)

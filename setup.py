import os
from setuptools import setup
from glob import glob

package_name = 'ros2_stretch_docking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # (os.path.join('share', package_name, 'config'), ['config/*.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Nan Fernandez-Ayala',
    maintainer_email='victor@animum.ai',
    description='Custom docking procedure using Nav2 and ArUco for Hello Robot Stretch',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # These console scripts can be run via `ros2 run ros2_stretch_docking aruco_detect_node`
            'aruco_detect_node = ros2_stretch_docking.aruco_detect_node:main',
            'docking_node = ros2_stretch_docking.docking_node:main',
            'docking_node_sim = ros2_stretch_docking.docking_node_sim:main',
        ],
    },
)
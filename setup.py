from setuptools import setup
import os
from glob import glob

package_name = 'px4_vision_autonomy'

# Helper to collect data files recursively
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'scripts'), glob('scripts/tools/*.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
]

# Add models recursively
for root, dirs, files in os.walk('models'):
    if files:
        destination = os.path.join('share', package_name, root)
        source_paths = [os.path.join(root, f) for f in files]
        data_files.append((destination, source_paths))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes'],
    data_files=data_files,
    install_requires=['setuptools', 'mavsdk', 'opencv-contrib-python', 'numpy'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='A ROS2 Jazzy package for PX4 offboard control with computer vision',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_viewer = px4_vision_autonomy.nodes.camera_viewer:main',
            'aruco_detector = px4_vision_autonomy.nodes.aruco_detector:main',
            'mission_commander = px4_vision_autonomy.nodes.mission_commander:main',
        ],
    },
)

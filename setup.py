import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hex_ros_demo_joystick'


def get_files(target, source):
    """Return install entries for files in a package-relative directory."""
    files = []
    source_path = os.path.join(os.path.dirname(__file__), source)
    for path in glob(os.path.join(source_path, '*')):
        if os.path.isfile(path):
            files.append((target, [path]))
    return files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_files('share/' + package_name, 'launch/ros2'),
        *get_files('share/' + package_name + '/config/ros1', 'config/ros1'),
        *get_files('share/' + package_name + '/config/ros2', 'config/ros2'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dong Zhaorui',
    maintainer_email='dzr159@gmail.com',
    description='Joystick-controlled MuJoCo arm demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_arm = hex_ros_demo_joystick.joy_arm:main',
        ],
    },
)

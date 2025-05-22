from setuptools import find_packages
from setuptools import setup

setup(
    name='dual_camera_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('dual_camera_msgs', 'dual_camera_msgs.*')),
)

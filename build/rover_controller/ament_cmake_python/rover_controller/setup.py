from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_controller',
    version='0.0.0',
    packages=find_packages(
        include=('rover_controller', 'rover_controller.*')),
)

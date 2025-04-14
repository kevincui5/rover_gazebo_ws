from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_utils',
    version='0.0.0',
    packages=find_packages(
        include=('rover_utils', 'rover_utils.*')),
)

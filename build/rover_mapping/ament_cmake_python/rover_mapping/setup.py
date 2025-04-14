from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_mapping',
    version='0.0.0',
    packages=find_packages(
        include=('rover_mapping', 'rover_mapping.*')),
)

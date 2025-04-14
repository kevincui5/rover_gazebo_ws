from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_planning',
    version='0.0.0',
    packages=find_packages(
        include=('rover_planning', 'rover_planning.*')),
)

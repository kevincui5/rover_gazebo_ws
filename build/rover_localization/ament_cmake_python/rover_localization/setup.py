from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_localization',
    version='0.0.0',
    packages=find_packages(
        include=('rover_localization', 'rover_localization.*')),
)

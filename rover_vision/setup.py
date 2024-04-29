#!/usr/bin/env python3
from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['classes_vision'],
    package_dir = {'': 'scripts'}
)

setup(**setup_args) 
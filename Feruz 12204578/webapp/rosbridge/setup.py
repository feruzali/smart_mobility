#!/usr/bin/env python

# Author: Feruz 12204578

## File to setup the rosbridge_server

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roswww'],
    package_dir={'': 'src'},
)

setup(**d)

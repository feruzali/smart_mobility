"""
Overview:
Python setup script for the TurtleBot3 example package.

Note:
This script is used to fetch values from the package.xml and set up the Python package using distutils.

"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['turtlebot3_example'],
    package_dir={'': 'src'}
)

# Set up the Python package
setup(**setup_args)

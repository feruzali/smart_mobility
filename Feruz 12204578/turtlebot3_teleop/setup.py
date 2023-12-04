"""
Author: Feruz 12204578

ROS Package Setup Script

This script is part of the setup process for a ROS (Robot Operating System) package.
It uses distutils and catkin_pkg to configure the distribution of the package.

Overview:
1. Imports necessary functions from distutils.core and catkin_pkg.python_setup.
2. Utilizes generate_distutils_setup to fetch package information from the associated package.xml file.
3. Specifies the Python packages to be installed (packages) and their directory structure (package_dir).
4. Calls the setup function with the generated setup configuration to install the Python package.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml and generate setup configuration
setup_args = generate_distutils_setup(
    packages=['turtlebot3_teleop'],  # List of Python packages to be installed
    package_dir={'': 'src'}           # Directory structure of the Python packages
)

# Call the setup function with the generated setup configuration
setup(**setup_args)

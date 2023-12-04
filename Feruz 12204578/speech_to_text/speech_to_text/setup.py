"""
Author: Feruz 12204578

ROS Package Setup Script

This Python script serves as the 'setup.py' file for packaging the ROS (Robot Operating System) package 'speech_to_text'.
It includes metadata, resource files, dependencies, and console scripts necessary for the installation and execution of the package.

Overview:
1. Define the package name as "speech_to_text".
2. Configure package metadata such as name, version, author, and description.
3. Specify package dependencies and installation details.
4. Define the installation paths for resource files, including grammars, launch files, and package.xml.
5. Set up console scripts (ROS nodes) to be installed, allowing for easy execution of various nodes.

"""

from glob import glob
import os
from setuptools import setup

# Define the package name
package_name = "speech_to_text"

# Setup configuration
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        # Install resource files and package.xml
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # Install grammars, launch files, and launch XML files
        (os.path.join("share", package_name, "grammars"), glob("grammars/*.gram")),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("launch/*.launch.xml"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Feruz",
    author_email="feruz.privet@gmail.com",
    description="TODO: Package description",  # Update with the actual package description
    license="TODO: License declaration",        # Update with the actual license
    tests_require=["pytest"],
    entry_points={
        # Define console scripts (ROS nodes) to be installed
        "console_scripts": [
            "stt_node = speech_to_text.stt_node:main",
            "nlp_node = speech_to_text.nlp_node:main",
            "parser_node = speech_to_text.parser_node:main",
            "dialog_manager_node = speech_to_text.dialog_manager_node:main"
        ],
    },
)

# setup.py

from setuptools import setup, find_packages

setup(
    name='waiter_bot',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'rospy',
        'click',
    ],
    entry_points={
        'console_scripts': [
            'waiter_bot = waiter_bot:run_waiter_bot',
        ],
    },
)

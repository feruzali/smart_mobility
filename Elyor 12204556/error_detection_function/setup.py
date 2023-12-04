from setuptools import setup

package_name = 'error_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['rospy', 'std_msgs'], 
    author='elyor',  
    author_email='elyor9760@gmail.com',
    maintainer='elyor', 
    maintainer_email='elyor9760@gmail.com',
    description='The error_detection package',
    license='TODO',  
    keywords=['ROS', 'Robotics'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License', 
        'Programming Language :: Python :: 2.7',  
        'Programming Language :: Python :: 3.6', 
        'Topic :: Software Development',
    ],
)


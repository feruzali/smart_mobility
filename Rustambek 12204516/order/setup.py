from setuptools import find_packages, setup

package_name = 'order'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rustam',
    maintainer_email='rrakhimov64@gmail.com',
    description='Package for sending orders to the server.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = order.publisher_member_function:main',
                'listener = order.subscriber_member_function:main',
                'menu=order.menu:run',
        ],
    },
)

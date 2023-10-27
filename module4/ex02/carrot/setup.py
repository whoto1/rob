from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carrot'

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
    maintainer='alina',
    maintainer_email='a.ten3@g.nsu.ru',
    description='learning tf2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'turtle_tf2_broadcaster = my_carrot.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = my_carrot.turtle_tf2_listener:main',
            'dynamic = my_carrot.dynamic_frame_tf2_broadcaster:main',
        ],
    },
)

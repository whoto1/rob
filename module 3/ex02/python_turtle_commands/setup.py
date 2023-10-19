from setuptools import find_packages, setup

package_name = 'python_turtle_commands'

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
    description='Package action',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'server = python_turtle_commands.action_server:main',
        	'client = python_turtle_commands.action_client:main',
        ],
    },
)

try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup, find_packages

package_name = 'aws_robomaker_simulation_common'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(), 
     data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('lib/' + package_name, ['src/aws_robomaker_simulation_common/route_manager.py']),
        ('share/' + package_name, ['package.xml']),
     ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'AWS RoboMaker - Send goals to move_base server for the specified route. Routes forever.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_manager = route_manager:main',
        ],
    },    
)

from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('cartographer_config/*')),
        (os.path.join('share', package_name), glob('slam_pkg_config/*')),
        (os.path.join('share', package_name), glob('nav2_params/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antony',
    maintainer_email='antonikaras1995@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simpleRobot = autonomous_exploration.simpleRobot:main',
            'autonomousExploration = autonomous_exploration.autonomousExploration:main',
            'rosbridge_msgs_publisher = autonomous_exploration.rosbridge_msgs_publisher:main',
        ],
    },
)

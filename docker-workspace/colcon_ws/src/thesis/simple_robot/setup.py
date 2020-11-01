from setuptools import setup, find_packages

package_name = 'simple_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='antonikaras1995@hotmail.com',
    description='A simple terminal controller for the turtlebot3 robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simpleRobot = simple_robot.simpleRobot:main',
        ],
    },
)  
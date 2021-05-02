from setuptools import setup
import os
from glob import glob

package_name = 'interactive_map_tester'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('params/*')),
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
            'visualizeInteractiveMap = interactive_map_tester.visualizeInteractiveMap:main',
            'saveInteactiveMap = interactive_map_tester.saveInteractiveMap:main',
            'loadInteactiveMap = interactive_map_tester.loadInteractiveMap:main',
            'inteactiveMapDemo = interactive_map_tester.interactiveMapDemo:main'
        ],
    },
)

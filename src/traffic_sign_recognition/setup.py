from setuptools import setup
import os
from glob import glob

package_name = 'traffic_sign_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
     data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeduthun Idemudia',
    maintainer_email='n1044024@my.ntu.ac.uk',
    description='Traffic sign recognition package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'traffic_sign_node = traffic_sign_recognition.traffic_sign_node:main',
		'decision_maker_node = traffic_sign_recognition.decision_maker_node:main',
        ],
    },
)

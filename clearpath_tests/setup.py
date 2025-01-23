from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'clearpath_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Iverach-Brereton',
    maintainer_email='civerachb@clearpathrobotics.com',
    description='Testing scripts for Clearpath robots',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpu_stress = clearpath_tests.cpu_stress:main',
            'drive_test = clearpath_tests.drive_test:main',
            'fan_test = clearpath_tests.fan_test:main',
            'light_test = clearpath_tests.light_test:main',
            'rotation_test = clearpath_tests.rotation_test:main',
        ],
    },
)

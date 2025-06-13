from setuptools import setup
import os
from glob import glob

package_name = 'disparity_extender'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='champion',
    maintainer_email='champion@f1tenth.org',
    description='Disparity Extender F1TENTH Championship Algorithm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'disparity_extender = disparity_extender.scripts.disparity_extender:main',
        ],
    },
)

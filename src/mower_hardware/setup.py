from glob import glob
import os

from setuptools import setup


package_name = 'mower_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yh',
    maintainer_email='yh@example.com',
    description='Hardware bring-up profiles for the mower platform.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rtk_ntrip_node = mower_hardware.rtk_ntrip_node:main',
        ],
    },
)

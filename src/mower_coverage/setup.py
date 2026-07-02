from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'mower_coverage'

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
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yh',
    maintainer_email='yh@example.com',
    description='全覆盖路径规划包 — 自动割草机牛耕式路径规划',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boustrophedon_planner = mower_coverage.boustrophedon_planner:main',
            'area_definer = mower_coverage.area_definer:main',
            'path_executor = mower_coverage.path_executor:main',
            'coverage_monitor = mower_coverage.coverage_monitor:main',
            'coverage_demo = mower_coverage.coverage_demo:main',
        ],
    },
)

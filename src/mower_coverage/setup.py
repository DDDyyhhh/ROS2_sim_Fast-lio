from setuptools import setup
from setuptools import find_packages
import os
from glob import glob
import fnmatch


def _find_files(directory, pattern):
    """递归查找文件"""
    matches = []
    for root, dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, pattern):
            matches.append(os.path.join(root, filename))
    return matches

package_name = 'mower_coverage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web_frontend'),
         [f for f in _find_files('web_frontend', '*') if os.path.isfile(f)]),
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
            'boustrophedon_planner = mower_coverage.legacy.boustrophedon_planner:main',
            'area_definer = mower_coverage.legacy.area_definer:main',
            'path_executor = mower_coverage.legacy.path_executor:main',
            'coverage_monitor = mower_coverage.legacy.coverage_monitor:main',
            'coverage_demo = mower_coverage.legacy.coverage_demo:main',
            # === 斜坡场景 + 多区域新增节点 ===
            'multi_area_definer = mower_coverage.mission.multi_area_definer:main',
            'hill_boustrophedon = mower_coverage.planning.hill_boustrophedon:main',
            'multi_area_executor = mower_coverage.execution.multi_area_executor:main',
            'web_server = mower_coverage.adapters.web_server:main',
        ],
    },
)

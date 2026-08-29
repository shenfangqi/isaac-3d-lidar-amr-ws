from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'isaac_3d_lidar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config/nvblox'),
            glob('config/nvblox/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shenfq',
    maintainer_email='shenfq@todo.todo',
    description='Isaac Sim XT32 Nvblox Bringup',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'amcl_pose_initializer = '
            'isaac_3d_lidar_bringup.amcl_pose_initializer:main',
            'pointcloud_padder = '
            'isaac_3d_lidar_bringup.pointcloud_padder:main',
        ],
    },
)

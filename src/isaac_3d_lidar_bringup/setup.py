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
        (
            os.path.join('share', package_name, 'config/slam_toolbox'),
            glob('config/slam_toolbox/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config/state_estimation'),
            glob('config/state_estimation/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config/nav2'),
            glob('config/nav2/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shenfq',
    maintainer_email='shenfq@todo.todo',
    description='Isaac Sim 3D lidar and nvblox bringup',
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
            'pointcloud_evidence_degrader = '
            'isaac_3d_lidar_bringup.pointcloud_evidence_degrader:main',
            'overhead_clearance_marker_publisher = '
            'isaac_3d_lidar_bringup.'
            'overhead_clearance_marker_publisher:main',
            'static_map_scan_filter = '
            'isaac_3d_lidar_bringup.static_map_scan_filter:main',
            'mesh_voxel_relay = '
            'isaac_3d_lidar_bringup.mesh_voxel_relay:main',
            'fast_lio_base_adapter = '
            'isaac_3d_lidar_bringup.fast_lio_base_adapter:main',
        ],
    },
)

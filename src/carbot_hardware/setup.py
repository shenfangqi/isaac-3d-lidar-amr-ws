from glob import glob
from setuptools import find_packages, setup


package_name = "carbot_hardware"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/systemd",
            glob("systemd/*.service") + glob("systemd/*.cfg"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Carbot Maintainer",
    maintainer_email="maintainer@example.com",
    description="Host-side hardware integration for the Carbot tracked base.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mid360_imu_adapter = carbot_hardware.mid360_imu_adapter:main",
            "pointcloud_xyz_relay = carbot_hardware.pointcloud_xyz_relay:main",
            "wheel_odometry = carbot_hardware.wheel_odometry:main",
        ],
    },
)

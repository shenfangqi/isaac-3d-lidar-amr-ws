from setuptools import find_packages, setup


package_name = 'carbot_nav_recovery'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shenfq',
    maintainer_email='shenfq@todo.todo',
    description=(
        'Geometry safety evaluation for bounded Carbot navigation recovery'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recovery_validation_visualizer = '
            'carbot_nav_recovery.validation_visualizer:main',
            'recovery_runtime_observer = '
            'carbot_nav_recovery.runtime_observer:main',
            'recovery_coordinator = '
            'carbot_nav_recovery.recovery_coordinator:main',
            'complex_route_advisor = '
            'carbot_nav_recovery.complex_route_advisor:main',
            'analyze_complex_route_evidence = '
            'carbot_nav_recovery.analyze_complex_route_evidence:main',
        ],
    },
)

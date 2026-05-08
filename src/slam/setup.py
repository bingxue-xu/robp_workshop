from setuptools import find_packages, setup

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='arian.kourangi@hotmail.se',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'repeater = slam.repeater:main',
            'slam = slam.slam:main',
            'cv= slam.cv:main',
            'odometry = slam.odometry:main',
            'global_map= slam.global_map:main',
            'drive= slam.drive:main',
            'publish= slam.publisher:main',
            'scan_matching= slam.scan_matching:main',
            'local_map= slam.local_map:main',
            'object_handler= slam.object_handler:main',
            'marker_detect= slam.marker_detect:main',
        ],
    },
)

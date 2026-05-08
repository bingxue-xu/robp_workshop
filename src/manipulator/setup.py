from setuptools import find_packages, setup

package_name = 'manipulator'

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
    maintainer_email='axbr@kth.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_up_service = manipulator.pick_up_service:main',
             'place_service = manipulator.place_service:main',
             'test_client_node = manipulator.test_client_node:main',
             'invers_kinematics = manipulator.invers_kinematics:main',
             'posestamped_pub = manipulator.posestamped_pub:main'
        ],
    },
)

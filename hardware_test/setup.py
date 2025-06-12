from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'hardware_test'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('hardware_test', 'launch', '*.launch.py'))))
data_files.append((os.path.join('share', package_name, 'rviz'), glob(os.path.join('hardware_test', 'rviz', '*.rviz'))))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bingxue Xu',
    maintainer_email='autoxue@gmail.com',
    description='Test hardware components of the robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_test = hardware_test.base_test:main',
            'realsense_check = hardware_test.realsense_check:main',
            'rplidar_check = hardware_test.rplidar_check:main',
            'phidgets_check = hardware_test.phidgets_check:main',
            'usb_cam_check = hardware_test.usb_cam_check:main',
            'arm_check = hardware_test.arm_check:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'detection'

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
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = detection.detection:main',
            'detector = detection.detector:main',
            'utils = detection.utils:main',
            'classification = detection.classificationv1:main',
            'usb_cam_service = detection.usb_cam_service:main',
            'move_arm_service = detection.move_arm:main',
            'claw_detect = detection.arm_detection:main'
        ],
    },
)

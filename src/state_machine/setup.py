from setuptools import find_packages, setup

package_name = 'state_machine'

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
    maintainer_email='autoxue@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ms2 = state_machine.ms2_state_machine:main',
            'tmp_service_detection = state_machine.tmp_service_detection:main',
            'bt2 = state_machine.bt2_behavior_tree:main',
            'bt2_behaviors = state_machine.bt2_behaviors:main'

        ],
    },
)

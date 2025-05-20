import os
from glob import glob
from setuptools import setup

package_name = 'kobuki_description'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'rviz'), glob(
    os.path.join('rviz', '*.rviz'))))
data_files.append(('share/' + package_name + '/meshes/images', ['meshes/images/main_body.jpg']))
data_files.append(('share/' + package_name + '/meshes/images', ['meshes/images/wheel.jpg']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/main_body.dae']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/wheel.dae']))
data_files.append((os.path.join('share', package_name, 'urdf'), glob(
    os.path.join('urdf', '*'))))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Duberg',
    maintainer_email='dduberg@kth.se',
    description='kobuki_description',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
        ],
    },
)

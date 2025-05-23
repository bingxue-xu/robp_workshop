from setuptools import find_packages, setup

package_name = 'display_markers'

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
    maintainer='bingxue',
    maintainer_email='autoxue@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_markers = display_markers.display_markers:main',
            'display = display_markers.display:main',
            'display_point = display_markers.display_point:main',
        ],
    },
)

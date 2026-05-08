from setuptools import find_packages, setup

package_name = 'exploration'

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
    maintainer_email='gustav.forsell@live.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = exploration.explorer:main',
            'test_map = exploration.test_map:main',
            'test_viz = exploration.test_viz_client:main',
            'test_explorer = exploration.test_explorer_client:main',
        ],
    },
)

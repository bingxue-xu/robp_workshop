from setuptools import find_packages, setup

package_name = 'path_planning'

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
            'global_planning = path_planning.global_planning:main',
            'local_planning = path_planning.local_planning:main',
            'test_client = path_planning.test_client:main',
            'a_star = path_planning.a_star:main',
            'd_star_lite = path_planning.d_star_lite:main'

        ],
    },
)

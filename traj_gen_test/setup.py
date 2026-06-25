import os
from glob import glob
from setuptools import setup

package_name = 'traj_gen_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anno',
    maintainer_email='',
    description='Dummy data publisher to exercise ros_traj_gen_utils.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_publisher = traj_gen_test.dummy_publisher:main',
        ],
    },
)

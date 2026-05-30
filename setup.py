import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gps_heading_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tejaswi',
    maintainer_email='tejaswi.arun@gmail.com',
    description='Convert ublox moving-baseline GPS heading (UBXNavRelPosNED) into a '
                'standard sensor_msgs/Imu on /rover/imu for robot_localization.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'gps_heading_imu = {package_name}.heading_to_imu_node:main',
        ],
    },
)

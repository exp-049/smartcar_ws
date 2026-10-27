from glob import glob
from setuptools import find_packages, setup

package_name = 'smartcar_sensor_hal'

setup(
    name=package_name,
    version='0.6.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Smartcar project',
    maintainer_email='maintainers@example.invalid',
    description='ESP32-S3 UDP lidar and MPU6050 to ROS 2 with qualified time synchronization',
    license='Apache-2.0',
    entry_points={'console_scripts': ['sensor_hal = smartcar_sensor_hal.node:main']},
)

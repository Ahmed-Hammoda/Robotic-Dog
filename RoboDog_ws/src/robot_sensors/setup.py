from setuptools import setup
from glob import glob

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='ROS 2 sensor nodes for Raspberry Pi 5, including MPU9250 fall detection and RViz visualization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aht10_node = robot_sensors.aht10_node:main',
            'mpu9250_node = robot_sensors.mpu9250_node:main',
        ],
    },
)

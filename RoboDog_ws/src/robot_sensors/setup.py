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
    description='ROS 2 sensor node for Raspberry Pi 5 reading ultrasonic, AHT10, and GY-87 sensors.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_node = robot_sensors.ultrasonic_node:main',
            'ultrasonic_range_bridge_node = robot_sensors.ultrasonic_range_bridge_node:main',
            'ultrasonic_occupancy_node = robot_sensors.ultrasonic_occupancy_node:main',
            'ultrasonic_occupancy_mapper_node = robot_sensors.ultrasonic_occupancy_mapper_node:main',
            'ultrasonic_test_publisher = robot_sensors.ultrasonic_test_publisher:main',
            'imu_simulator_node = robot_sensors.imu_simulator_node:main',
            'aht10_node = robot_sensors.aht10_node:main',
            'mpu9250_node = robot_sensors.mpu9250_node:main',
        ],
    },
)

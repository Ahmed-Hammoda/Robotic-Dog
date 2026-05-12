from setuptools import find_packages, setup
import os

package_name = 'robodog_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/camera_detection.launch.py'
        ]),
        ('share/' + package_name + '/models', [
            'models/yolov8n.ncnn.param',
            'models/yolov8n.ncnn.bin',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robodog',
    maintainer_email='robodog@robodog.local',
    description='RoboDog vision module with camera capture and NCNN object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_detection_node = robodog_vision.camera_detection_node:main',
            'ncnn_detection_node = robodog_vision.ncnn_detection_node:main',
        ],
    },
)

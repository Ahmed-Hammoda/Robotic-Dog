from setuptools import setup

package_name = 'robodog_stm32_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='STM32 bridge for motor commands.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = robodog_stm32_bridge.stm32_bridge_node:main',
        ],
    },
)

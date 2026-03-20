from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'biped_driver'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'python-can>=4.0.0',
        'numpy>=1.21.0',
    ],
    zip_safe=True,
    maintainer='Abhinav Roy',
    maintainer_email='abhinav.roy@cleanelectric.in',
    description='Hardware drivers for biped robot (RobStride motors + BNO085 IMU)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = biped_driver.imu_node:main',
            'can_bus_node = biped_driver.can_bus_node:main',
            'can_bus_node_async = biped_driver.can_bus_node_async:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'biped_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Roy',
    maintainer_email='abhinav.roy@cleanelectric.in',
    description='Keyboard and gamepad teleop for biped robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = biped_teleop.keyboard_teleop:main',
        ],
    },
)

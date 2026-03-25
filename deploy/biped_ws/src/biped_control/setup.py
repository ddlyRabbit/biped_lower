from setuptools import find_packages, setup

package_name = 'biped_control'

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
    description='Policy inference, safety, and state machine',
    license='MIT',
    entry_points={
        'console_scripts': [
            'policy_node = biped_control.policy_node:main',
            'safety_node = biped_control.safety_node:main',
            'state_machine_node = biped_control.state_machine_node:main',
            'zmp_trajectory_node = biped_control.zmp.zmp_trajectory_node:main',
        ],
    },
)

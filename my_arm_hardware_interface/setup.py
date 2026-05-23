from setuptools import setup
from setuptools import find_packages

package_name = 'my_arm_hardware_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lai',
    maintainer_email='123@qq.com',
    description='Hardware interface package for my_arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_pos_control = my_arm_hardware_interface.joint_pos_control:main',
            'hardware_interface = my_arm_hardware_interface.hardware_interface:main',
        ],
    },
)

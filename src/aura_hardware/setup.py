from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aura_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akk',
    maintainer_email='akk@todo.todo',
    description='Hardware drivers and data collection for AURA SO-ARM101',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'servo_driver             = aura_hardware.servo_driver:main',
            'servo_hardware_interface = aura_hardware.servo_hardware_interface:main',
            'camera_node              = aura_hardware.camera_node:main',
            'recorder_node            = aura_hardware.recorder_node:main',
            'gripper_node             = aura_hardware.gripper_node:main',
            'inference_node           = aura_hardware.inference_node:main',
            'smolvla_inference_node   = aura_hardware.smolvla_inference_node:main',
            'act_inference_node       = aura_hardware.act_inference_node:main',
            'smolvla_recorder_node    = aura_hardware.smolvla_recorder_node:main',
            'keyboard_teleop_node     = aura_hardware.keyboard_teleop_node:main',
            'digital_twin_node        = aura_hardware.digital_twin_node:main',
            'twin_teleop_node         = aura_hardware.twin_teleop_node:main',
        ],
    },
)

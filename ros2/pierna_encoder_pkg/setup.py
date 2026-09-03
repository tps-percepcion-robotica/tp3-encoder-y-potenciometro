import os
from glob import glob
from setuptools import setup

package_name = 'pierna_encoder_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francisco',
    maintainer_email='francisco@example.com',
    description='TP3 - pierna izquierda: potenciometro + AS5600 via micro-ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibracion_node = pierna_encoder_pkg.calibracion_node:main',
            'calibracion_node_encoder = pierna_encoder_pkg.calibracion_node_encoder:main',
        ],
    },
)

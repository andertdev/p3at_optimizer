import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'p3at_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Webots world/proto (opcional mas recomendado para organização)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'protos'), glob('protos/*.proto')),
        (os.path.join('share', package_name, 'protos', 'icons'), glob('protos/icons/*')),
        # Controller do Webots (para você manter versionado junto do projeto)
        (os.path.join('share', package_name, 'controllers', 'p3at_webots_controller'),
         glob('controllers/p3at_webots_controller/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iartes',
    maintainer_email='voce@email.com',
    description='P3AT Control Stack (SIM + REAL)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Runtime nodes in active pipeline
            'executor = p3at_control.p3at_executor:main',
            'api = p3at_control.p3at_api:main',
            'bus_udp = p3at_control.p3at_bus_udp:main',
        ],
    },
)

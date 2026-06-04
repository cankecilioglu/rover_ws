import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'rover_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c4nth',
    maintainer_email='cankecilioglu@gmail.com',
    description='Proprioceptive terrain heatmap from IMU + odometry',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'terrain_heatmap_node = rover_mapping.terrain_heatmap_node:main',
            'map_autosave_node = rover_mapping.map_autosave_node:main',
        ],
    },
    
)

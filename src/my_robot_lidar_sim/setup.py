from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_lidar_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzw',
    maintainer_email='Zzw2600@outlook.com',
    description='CPU 光线投射 2D 激光雷达（替代 Gazebo GPU 雷达）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cpu_lidar_node = my_robot_lidar_sim.cpu_lidar_node:main',
        ],
    },
)

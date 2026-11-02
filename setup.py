from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_web_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('ros_web_monitor/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Lightweight ROS 2 topic and camera monitoring dashboard.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'monitor_node = ros_web_monitor.monitor_node:main',
        ],
    },
)

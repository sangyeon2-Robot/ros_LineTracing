from setuptools import find_packages, setup

package_name = 'ros_linetracing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/linetracing_with_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='https://github.com/sangyeon2-Robot/ros_mega.git',
    description='Line tracing node with manual/auto switching and serial V,w output.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_camera_node = ros_linetracing.line_camera_node:main',
            'serial_bridge_node = ros_linetracing.serial_bridge_node:main',
            'teleop_serial_node = ros_linetracing.teleop_serial_node:main',
        ],
    },
)

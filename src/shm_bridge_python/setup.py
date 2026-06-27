from setuptools import find_packages, setup

package_name = 'shm_bridge_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raj',
    maintainer_email='47215482+rajpcrj@users.noreply.github.com',
    description='Python nodes and importable library for a generic type-agnostic ROS 2 to shared-memory bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_rgb_to_shm = shm_bridge_python.ros_rgb_to_shm:main',
            'shm_rgb_viewer = shm_bridge_python.shm_rgb_viewer:main',
            'ros2_udp_streamer = shm_bridge_python.ros2_udp_streamer:main',
            'generic_shm_writer = shm_bridge_python.generic_shm_writer:main',
            'generic_shm_reader = shm_bridge_python.generic_shm_reader:main',
            'ros2_to_shm = shm_bridge_python.ros2_to_shm:main',
            'ros2_to_shm_all = shm_bridge_python.ros2_to_shm_all:main',
            'example_usage = shm_bridge_python.example_usage:main',
        ],
    },
)

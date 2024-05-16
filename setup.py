from setuptools import setup
package_name = 'ros_micro_bridge'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/default_outside_stack.launch.py']),
        ('share/' + package_name, ['launch/gnss_bringup.launch.py']),
        ('share/' + package_name, ['launch/low_lvl_bridge.launch.py']) 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='casper',
    maintainer_email='casper.cromjongh@hotmail.com',
    description='The bridge between the Arduino and ROS on the NUC using Python',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_ros2 = ros_micro_bridge.arduino_bridge_ros2:main',
        ],
    },
)

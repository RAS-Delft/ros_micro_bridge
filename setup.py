import os
package_name = 'ras_low_level_bridge'

if os.getenv("ROS_VERSION") == "1":
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    setup_args = generate_distutils_setup(
        packages=['ras_low_level_bridge'],
        # Package directories can be specified here, with format 'package-you-want': 'directory-to-look'
        # https://docs.python.org/3/distutils/setupscript.html#listing-whole-packages
        # Here not required since we just look in the project root
        # package_dir={'': ''},
    )

    setup(**setup_args)

else:
    from setuptools import setup

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
                'arduino_bridge_ros2 = ras_low_level_bridge.arduino_bridge_ros2:main',
            ],
        },
    )

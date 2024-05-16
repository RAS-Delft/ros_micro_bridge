import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
import os

def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()


    vessel_id = os.getenv('VESSEL_ID', 'nameless_vessel')

    # Define the nmea_tcp_driver node
    nmea_tcp_driver_node = Node(
        package='reach_ros_node',
        executable='nmea_tcp_driver',
        name='gnss_bridge',
        namespace=vessel_id,
        output='screen',
        parameters=[
            {'host': '192.168.2.15'},
            {'port': '9001'},
        ],
        remappings=[
            ('tcpfix', 'telemetry/gnss/fix')
        ]
    )
    ld.add_action(nmea_tcp_driver_node)

    return ld
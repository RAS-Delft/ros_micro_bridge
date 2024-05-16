import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Get the VESSEL_ID environment variable, if non-existent, default to 'nameless_vessel'
    vessel_id = os.getenv('VESSEL_ID', 'nameless_vessel')

    # Define the arduino_bridge_ros2 node
    arduino_bridge_node = Node(
        package='ras_low_level_bridge',
        executable='arduino_bridge_ros2',
        name='arduino_bridge',
        output='screen',
        namespace=vessel_id,
        remappings=[
            ('state/yaw', 'telemetry/heading')
        ]
    )
    ld.add_action(arduino_bridge_node)

    return ld
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    # start gnss_bringup.launch.py
    gnss_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ras_low_level_bridge'), 'launch'),
         '/gnss_bringup.launch.py'])
    )
    ld.add_action(gnss_launch)

    # start lowl_lvl_bridge.launch.py
    lowl_lvl_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ras_low_level_bridge'), 'launch'),
         '/low_lvl_bridge.launch.py'])
    )
    ld.add_action(lowl_lvl_launch)

    return ld
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    display_amv_launch_file = os.path.join(
        get_package_share_directory('pilz_tutorial'), 'launch')
    kinematics_launch_file = os.path.join(
        get_package_share_directory('pilz_tutorial'), 'configs/kinematics')
    rviz_config_path = os.path.join(
        get_package_share_directory('pilz_tutorial'),'configs/rviz/display_amv.rviz')
    
    psen_scan_launch_file = os.path.join(
        get_package_share_directory("psen_scan_v2"), 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([display_amv_launch_file, '/display_amv.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([kinematics_launch_file, '/kinematics.launch.py'])
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            name='teleop_twist_keyboard',
            prefix="xterm -e"
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        ),
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource([psen_scan_launch_file, '/bringup.launch.xml'])
        # ),
    ])
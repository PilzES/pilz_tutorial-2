import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    kinematics_config_file = os.path.join(
        get_package_share_directory('pilz_tutorial'),'configs/kinematics','kinematics.yaml')

    return LaunchDescription([
        Node(
            package='differential_kinematics',
            executable='differential_kinematics_node',
            output='screen',
            name='differential_kinematics_node',
            parameters = [kinematics_config_file]
        ),

        Node(
            package='differential_kinematics',
            executable='drive_bridge_node',
            output='screen',
            name='drive_bridge_node',
        )
    ])
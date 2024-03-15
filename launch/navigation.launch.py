import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    launch_file = os.path.join(
        get_package_share_directory("pilz_tutorial"), "launch")

    rviz_config_path = os.path.join(
        get_package_share_directory("pilz_tutorial"), "configs/rviz/nav2_default_view.rviz")
    
    return LaunchDescription([

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file, "/bringup.launch.py"])
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file, "/bringup_amcl.launch.py"])
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file, "/bringup_nav2.launch.py"])
        ),

        Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments = ["-d", rviz_config_path]
        ),

        Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),

        Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]
        )
    ])
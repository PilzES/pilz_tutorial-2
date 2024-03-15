import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory 
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

  bringup_launch_file = os.path.join(
    get_package_share_directory("pilz_tutorial"), "launch")

  slam_config_file = LaunchConfiguration("parameters", default=os.path.join(
       get_package_share_directory("pilz_tutorial"), "configs/slam/slam.yaml"))

  rviz_config_path = os.path.join(
       get_package_share_directory("pilz_tutorial"), "configs/rviz/mapping_default_view.rviz")  
  
  return LaunchDescription([

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([bringup_launch_file, "/bringup.launch.py"])
    ),

    Node(
      package="slam_toolbox",
      executable="sync_slam_toolbox_node",
      name="slam_toolbox",
      output="screen",
      parameters = [slam_config_file]
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
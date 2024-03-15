from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

  urdf_path = PathJoinSubstitution([FindPackageShare("pilz_tutorial"), "urdf/pilz_amv.xacro"])
      
  return LaunchDescription([
    DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[
              {
                  'robot_description': Command(['xacro ', LaunchConfiguration("urdf")])
              }
          ]
      	)
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'publish_period': 0.001}]
        # ),
    ])
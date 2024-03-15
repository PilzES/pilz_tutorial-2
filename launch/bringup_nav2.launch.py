import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory("pilz_tutorial")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    
    lifecycle_nodes = ["controller_server",
                       "smoother_server",
                       "planner_server",
                       "behavior_server",
                       "bt_navigator",
                       "waypoint_follower",
                       "velocity_smoother"]

    remappings = [("/tf", "tf"),
                  ("/tf_static", "tf_static")]
    
    param_substitutions = {
      "use_sim_time": use_sim_time,
      "autostart": autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file = params_file,
            param_rewrites = param_substitutions,
            convert_types = True),
        allow_substs = True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false")
    
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file", default_value=os.path.join(bringup_dir, "configs/navigation", "navigation.yaml"))

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true")

    load_nodes = GroupAction(
        actions = [
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings + [("cmd_vel", "cmd_vel_nav")]),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments = ["--ros-args"],
                parameters = [{"use_sim_time": use_sim_time},
                              {"autostart": autostart},
                              {"node_names": lifecycle_nodes}])

        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(load_nodes)
    return ld
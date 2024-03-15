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
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    
    lifecycle_nodes = ["map_server", "amcl"]

    remappings = [("/tf", "tf"),
                  ("/tf_static", "tf_static")]

    param_substitutions = {
      "use_sim_time": use_sim_time,
      "yaml_filename": map_yaml_file}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file = params_file,
            param_rewrites = param_substitutions,
            convert_types = True),
        allow_substs = True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value=os.path.join(bringup_dir, "maps", "pilz_map_2.yaml"))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false")
    
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file", default_value=os.path.join(bringup_dir, "configs/navigation", "navigation.yaml"))

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true")

    load_nodes = GroupAction(
        actions = [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters = [configured_params],
                arguments = ["--ros-args"],
                remappings = remappings), 
            
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments = ["--ros-args"],
                parameters = [{"use_sim_time": use_sim_time},
                              {"autostart": autostart},
                              {"node_names": lifecycle_nodes}])

        ]
    )
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(load_nodes)
    return ld
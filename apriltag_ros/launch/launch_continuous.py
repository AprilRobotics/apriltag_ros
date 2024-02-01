import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = 'apriltag_ros'

def generate_launch_description():
    # Configs files
    config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")
    tags_config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "tags.yaml")

    # Launch arguments
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='False', 
        choices=['True', 'False'],
        description='Parameter use_sim_time')

    # Launch arguments
    sim_time = LaunchConfiguration('use_sim_time')

    # Nodes to launch
    node = Node(
        package=PACKAGE_NAME,
        executable='ContinuousDetector', # Other option SingleDetector
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[config_file],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                sim_time_arg,
                SetParameter("use_sim_time", sim_time),
                SetParameter("tags_yaml_path", tags_config_file),
                node
            ]
        )
    ])

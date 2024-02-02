import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = 'apriltag_ros'

def generate_launch_description():
    # Configuración de los archivos
    config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")
    tags_config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "tags.yaml")

    # Argumento de lanzamiento
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='False', 
        choices=['True', 'False'],
        description='Parameter use_sim_time')

    # Substitución de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodos para lanzar
    node = Node(
        package=PACKAGE_NAME,
        executable='single_image_server',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                use_sim_time_arg,
                SetParameter("use_sim_time", use_sim_time),
                SetParameter("tags_yaml_path", tags_config_file),
                node
            ]
        )
    ])
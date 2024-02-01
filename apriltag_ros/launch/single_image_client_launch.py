import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='single_image_client',
            output='screen',
            parameters=[
                {"fx": 652.7934615847107},
                {"fy": 653.9480389077635},
                {"cx": 307.1288710375904},
                {"cy": 258.7823279214385},
            ],
            remappings=[],
            arguments=[
                LaunchConfiguration('image_load_path'),
                LaunchConfiguration('image_save_path')
            ]
        ),
    ])
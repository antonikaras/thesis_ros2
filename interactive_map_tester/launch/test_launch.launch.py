
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    interactive_map_tester_dir = get_package_share_directory('interactive_map_tester')
    
    return LaunchDescription([

        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[interactive_map_tester_dir + '/map_server.yaml'])            
    ])
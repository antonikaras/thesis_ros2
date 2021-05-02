
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    interactive_map_tester_dir = get_package_share_directory('interactive_map_tester')
    
    cartographer_dir = get_package_share_directory('autonomous_exploration')
    interactiveMapTester_dir = get_package_share_directory('interactive_map_tester')
    cartographer_configuration = TURTLEBOT3_MODEL + '.lua'

    print(interactive_map_tester_dir)
    print(cartographer_configuration)

    return LaunchDescription([
    DeclareLaunchArgument('use_sim_time', default_value='true'),

     IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_dir + '/turtlebot3_cartographer.launch.py'),
        launch_arguments={'use_sim_time': 'true',
                          'cartographer_config_dir' : interactiveMapTester_dir,
                          'configuration_basename' : cartographer_configuration,
                          'cartographer_mode' : 'localization'}.items()
        )
                    
    ])
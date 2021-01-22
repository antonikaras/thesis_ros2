import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
    
def launch_setup(context, *args, **kwargs):

    # Define input variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model').perform(context)
    world = LaunchConfiguration('world').perform(context)
    world = 'turtlebot3_new_house.launch.py'

    # Add the turtlebot3 model
    os.environ["TURTLEBOT3_MODEL"] = turtlebot3_model

    # Add the simulation world launch file
    world_dir = get_package_share_directory('vision_based_exploration')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_dir + '/' + world),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Add the navigation2 launch file
    navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_dir + '/launch/navigation2.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    ) 

    # Add the SLAM launch file
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_dir + '/launch/online_async_launch.py'),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    
    return [world_launch, navigation2_launch, slam_toolbox_launch]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='house'),
        OpaqueFunction(function = launch_setup)
        ])
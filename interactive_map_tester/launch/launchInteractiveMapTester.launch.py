import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    # Define input variables
    map = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model').perform(context)
    world = LaunchConfiguration('world').perform(context)

    # Add the turtlebot3 model
    os.environ["TURTLEBOT3_MODEL"] = turtlebot3_model

    # Add the simulation world launch file
    world_dir = get_package_share_directory('thesis_gazebo')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_dir + '/launch/launch_simulation.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'gui':gui, 'world':world}.items()
    )
    
    # Add the navigation2 launch file
    navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_dir + '/launch/navigation2.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map}.items()
    ) 

    # Add the rosbridge_msgs publisher
    rosbridge_msgs_pub = Node(package='autonomous_exploration',
                          executable='rosbridge_msgs_publisher')
    
    # Start publishing the maps
    publish_maps = Node(package='interactive_map_tester',
                        executable='loadInteactiveMap')
    
    # Start the interactive_map_visualization node
    visualize_interactive_map = Node(package='interactive_map_tester',
                                     executable='visualizeInteractiveMap')

    return [world_launch, navigation2_launch, rosbridge_msgs_pub, visualize_interactive_map, publish_maps]

def generate_launch_description():
    default_map = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'turtlebot3_world.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='burger_new_house'),
        DeclareLaunchArgument("gui", default_value="True", description="Launch Gazebo UI?"),
        DeclareLaunchArgument("map", default_value=default_map, description="navigation_map"),
        OpaqueFunction(function = launch_setup)
        ])
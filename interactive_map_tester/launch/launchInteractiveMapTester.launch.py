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
    maps_folder = LaunchConfiguration('maps_folder').perform(context) + '/'
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model').perform(context)
    nav2_params_name = LaunchConfiguration('turtlebot3_model', default=turtlebot3_model).perform(context)
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
    nav2_param_dir = os.path.join(get_package_share_directory('autonomous_exploration'), nav2_params_name + '.yaml')
    navigation2_dir = get_package_share_directory('autonomous_exploration')
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_dir + '/turtlebot3_navigation2.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'params': nav2_param_dir}.items()
    ) 

    # Add the rosbridge_msgs publisher
    rosbridge_msgs_pub = Node(package='autonomous_exploration',
                          executable='rosbridge_msgs_publisher')
    
    # Start publishing the maps
    publish_maps = Node(package='interactive_map_tester',
                        executable='loadInteactiveMap',
                        parameters=[{"maps_folder" : maps_folder + world}])
    
    # Start the interactive_map_visualization node
    visualize_interactive_map = Node(package='interactive_map_tester',
                                     executable='visualizeInteractiveMap')

    # Add the cartographer launch file
    cartographer_dir = get_package_share_directory('autonomous_exploration')
    interactiveMapTester_dir = get_package_share_directory('interactive_map_tester')
    cartographer_configuration = turtlebot3_model + '.lua'
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_dir + '/turtlebot3_cartographer.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time,
                          'cartographer_config_dir' : interactiveMapTester_dir,
                          'configuration_basename' : cartographer_configuration,
                          'cartographer_mode' : 'localization'}.items()
        )
    
    # Pointcloud filter node
    pointcloud_filter = Node(package='pointcloud2_filter',
                                 executable='pcl_filter',
                                 parameters=[{
                                     'hBeams'           : 1875,
                                     'vBeams'           : 16,
                                     'vFoV'             : 0.2617993878,
                                     'hFoV'             : 3.1415,
                                     'robotBaseFrame'   : "velodyne_base_link",
                                     'sensorScanFrame'  : 'velodyne_base_scan'
                                 }])


    return [world_launch, navigation2_launch, cartographer_launch, rosbridge_msgs_pub, visualize_interactive_map, publish_maps, pointcloud_filter]

def generate_launch_description():
    default_map = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'turtlebot3_world.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('maps_folder', default_value='/home/antony/colcon_ws/src/thesis_ros2/maps'),
        DeclareLaunchArgument('nav2_parms',default_value='burger', description="name of the nav2 params"),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='new_house'),
        DeclareLaunchArgument("gui", default_value="True", description="Launch Gazebo UI?"),
        DeclareLaunchArgument("map", default_value=default_map, description="navigation_map"),
        OpaqueFunction(function = launch_setup)
        ])
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model').perform(context)
    mapping_package = LaunchConfiguration('mapping_package').perform(context)
    frontier_detection_method = LaunchConfiguration("frontier_detection_method").perform(context)
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
        launch_arguments={'use_sim_time': use_sim_time}.items()
    ) 

    # Add the SLAM launch file
    slam_toolbox_launch = Node(
        parameters=[
          get_package_share_directory("autonomous_exploration") + '/mapper_params_online_async.yaml',
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Add the cartographer launch file
    cartographer_dir = get_package_share_directory('autonomous_exploration')
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_dir + '/launch/turtlebot3_cartographer.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    
    # Choose the mapping algorithm
    if mapping_package == 'cartographer':
        mapping_package_launch = cartographer_launch
    else:
        mapping_package_launch = slam_toolbox_launch

    # Add the vision_based_frontier_detection
    vision_based_frontier_detection = Node(package = 'frontier_detection_vision',
                                           executable='frontierExplorationVision'
                                           )

    # Select the frontier detection method
    frontier_detection = vision_based_frontier_detection
    if frontier_detection_method == 'vision':
        frontier_detection = vision_based_frontier_detection

    # Add the autonomous_exploration action server
    autonomous_exploration_action_server = Node(package='autonomous_exploration',
                                                executable='autonomousExploration')

    return [world_launch, navigation2_launch, mapping_package_launch, frontier_detection, autonomous_exploration_action_server]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='burger_new_house'),
        DeclareLaunchArgument('mapping_package', default_value='slam'),
        DeclareLaunchArgument("gui", default_value="True", description="Launch Gazebo UI?"),
        DeclareLaunchArgument("frontier_detection_method", default_value="vision"),
        OpaqueFunction(function = launch_setup)
        ])
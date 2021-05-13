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
    nav2_params_name = LaunchConfiguration('turtlebot3_model', default=turtlebot3_model).perform(context)
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
    nav2_param_dir = os.path.join(get_package_share_directory('autonomous_exploration'), nav2_params_name + '.yaml')
    navigation2_dir = get_package_share_directory('autonomous_exploration')
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_dir + '/turtlebot3_navigation2.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'params': nav2_param_dir}.items()
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
        PythonLaunchDescriptionSource(cartographer_dir + '/turtlebot3_cartographer.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    
    # Choose the mapping algorithm
    if mapping_package == 'cartographer':
        mapping_package_launch = cartographer_launch
        pointCloud_to_laserScan_topic = 'points2'
    else:
        mapping_package_launch = slam_toolbox_launch
        pointCloud_to_laserScan_topic = 'points2'

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

    # Add the rosbridge_msgs publisher
    rosbridge_msgs_pub = Node(package='autonomous_exploration',
                          executable='rosbridge_msgs_publisher')
    
    # Add the simulation world launch file
    map_saver_dir = get_package_share_directory('nav2_map_server')
    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_saver_dir + '/launch/map_saver_server.launch.py')
    )
    
    # Start the interactive_map_visualization node
    visualize_interactive_map = Node(package='interactive_map_tester',
                                     executable='visualizeInteractiveMap')
    
    # Add the pointcloud_to_laserscan Node
    if turtlebot3_model == 'burger_velodyne':
        pointCloud_to_laserScan = Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', pointCloud_to_laserScan_topic)
                            ],
                parameters=[{
                    'target_frame'          : 'velodyne_base_scan',
                    'transform_tolerance'   : 0.01,
                    'min_height'            : 0.01,
                    'max_height'            : 3.0,
                    'angle_min'             : -3.1415,  # -M_PI/2
                    'angle_max'             : 3.1415,  # M_PI/2
                    'angle_increment'       : 0.0033504,  # M_PI/360.0
                    'scan_time'             : 0.005,
                    'range_min'             : 0.1,
                    'range_max'             : 70.0,
                    'use_inf'               : True,
                    'inf_epsilon'           : 1.0
                }],
                name='pointcloud_to_laserscan'
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

        return [world_launch, pointcloud_filter, mapping_package_launch, navigation2_launch, pointCloud_to_laserScan, autonomous_exploration_action_server, frontier_detection, rosbridge_msgs_pub, map_saver_launch, visualize_interactive_map]
        #return [world_launch, mapping_package_launch, navigation2_launch, pointCloud_to_laserScan, autonomous_exploration_action_server, frontier_detection, rosbridge_msgs_pub, map_saver_launch, visualize_interactive_map]
    else:
        return [world_launch, mapping_package_launch, navigation2_launch, autonomous_exploration_action_server, frontier_detection, rosbridge_msgs_pub, map_saver_launch, visualize_interactive_map] 

    #return [world_launch, mapping_package_launch, navigation2_launch, autonomous_exploration_action_server, rosbridge_msgs_pub, map_saver_launch, visualize_interactive_map]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav2_parms',default_value='burger', description="name of the nav2 params"),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='new_house'),
        DeclareLaunchArgument('mapping_package', default_value='cartographer'),
        DeclareLaunchArgument("gui", default_value="True", description="Launch Gazebo UI?"),
        DeclareLaunchArgument("frontier_detection_method", default_value="vision"),
        OpaqueFunction(function = launch_setup)
        ])
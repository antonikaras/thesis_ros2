import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):

    # Define input variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    world = os.path.join(get_package_share_directory('thesis_gazebo'),
                         'worlds', world_name, turtlebot3_model, turtlebot3_model + '.model')

    # Add the turtlebot3 model
    models_dir = os.path.join(get_package_share_directory('thesis_gazebo'), 'models')
    os.environ["TURTLEBOT3_MODEL"] = turtlebot3_model
    env1 = os.path.join(models_dir, 'factory', 'models') + ":"
    env2 = os.path.join(models_dir, 'hospital', 'models') + ":"
    env3 = os.path.join(models_dir, 'hospital', 'fuel_models') + ":"

    os.environ["GAZEBO_MODEL_PATH"] = models_dir + ":" + env1 + env2 + env3 + "$GAZEBO_MODEL_PATH"
    print(os.getenv("GAZEBO_MODEL_PATH"))
    
    # Robot state publisher
    urdf_file_name = 'turtlebot3_' + turtlebot3_model + '.urdf'
    urdf = os.path.join(
        get_package_share_directory('thesis_gazebo'),'models','turtlebot3_' + turtlebot3_model,
        'urdf',
        urdf_file_name)

    state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

    launch_file_dir = os.path.join(get_package_share_directory('thesis_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Add the gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )
    
    # Add the gzclient execution file
    gzclient_launch = ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui')))
    
    # Add the robot state publisher
    robot_state_pub_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    return [gazebo_launch, gzclient_launch, robot_state_pub_launch] 
    #return [robot_state_pub_launch] 

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='new_house'),
        OpaqueFunction(function = launch_setup)
        ])
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
                         'worlds', world_name, world_name + '.model')

    # Add the turtlebot3 model
    os.environ["TURTLEBOT3_MODEL"] = turtlebot3_model

    # Publish the directory of the additional model files
    models_dir = os.path.join(get_package_share_directory('thesis_gazebo'), 'models')
    if world_name == 'factory':
        print("-->", os.path.join(models_dir, 'factory', 'models') + ":$GAZEBO_MODEL_PATH")
        os.environ["GAZEBO_MODEL_PATH"] = os.path.join(models_dir, 'factory', 'models') + ":$GAZEBO_MODEL_PATH"
    elif world_name == 'hospital':
        os.environ["GAZEBO_MODEL_PATH"] = os.path.join(models_dir, 'hospital', 'models') + ":$GAZEBO_MODEL_PATH"
        #os.environ["GAZEBO_MODEL_PATH"] = os.path.join(models_dir, 'hospital', 'fuel_models') + ":$GAZEBO_MODEL_PATH"

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    print(world)
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

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('world', default_value='burger_new_house'),
        OpaqueFunction(function = launch_setup)
        ])
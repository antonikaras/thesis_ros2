# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def launch_setup(context, *args, **kwargs):

    # Define input variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    turtlebot3_cartographer_prefix = get_package_share_directory('autonomous_exploration')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default= TURTLEBOT3_MODEL + '.lua')
    cartographer_mode = LaunchConfiguration('cartographer_mode', default='mapping').perform(context)

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # Cartographer_node
    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])
    
    # Cartographer_grid_node
    cartographer_grid_node = Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
        
    # Start the cartographer nodes depending on the mode
    if cartographer_mode == "mapping":
        return [cartographer_node, cartographer_grid_node]
    elif cartographer_mode == "localization":
        return [cartographer_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('resolution', default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0',
            description='OccupancyGrid publishing period'),
        DeclareLaunchArgument('cartographer_mode', default_value='mapping'),

        OpaqueFunction(function = launch_setup)
        ])

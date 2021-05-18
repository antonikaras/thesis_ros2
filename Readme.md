# ROS2 turtlebot3 autonomous navigation

## Description

This package contains the ROS2 packages needed to implement autonomous exploration using the turtlebot3 robot. 
Also using ros1_bridge it connect with the Unity game Engine in order to visualize the map and make it interactive.

## Dependencies

* This package was implemented in ubuntu 20.04
* ROS2 foxy
* ROS noetic

## Installation instructions

* There is also a docker-image with the dependencies listed below pre-installed
    ``` git clone https://github.com/antonikaras/thesis_docker.git ```

1. Clone the ROS2 packages into your ROS2 workspace
```
mkdir ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/antonikaras/thesis_ros2.git
cd ../
colcon build --symlink-install
source install/setup.bash
```

2. Clone the ros1_bridge repository
    * It is recommended to create a separate colcon workspace for the rosbridge package
    * Compiling the ros1_bridge package requires some time (> 5 minutes)
```
mkdir -p bridge_ws/src
cd bridge_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge.git
cd ../
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
source install/setup.bash
```

3. Clone the ROS packages into the ros workspace
```
mkdir ~/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/antonikaras/thesis_ros.git
cd ../
catkin build
source devel/setup.bash
```

## Project packages

*   autonomous_exploration:
    1. Contains the autonomous exploration action server
    2. Handles the autonomous exploration using the new exploration targets detected by the specified frontier detection method
        i. Currently only vision based frontier detection is implemented
    3. Contains the simple robot api which is used to ease communication with the robot
* autonomous_exploration_msgs:
    1. Contains the message file definitions used by the frontier detection method
    2. Contains the action file definitions used by the autonomous exploration action 
    3. Contains the rosbridge_msgs used to connect ros2-ros-unity
* frontier_detection_vision:
    1. Contains the vision based frontier detection algorithm
* simulation_assets
    * thesis_gazebo:
        1. Contains the worlds and models used in the simulation
        2. Gazebo environments from ```https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps.git``` were used
        3. Velodyne VLP16 LiDAR was based on ``` https://github.com/ToyotaResearchInstitute/velodyne_simulator ```
    * velodyne_gazebo_plugins
        1. Contains the plugin for the velodyne LiDAR
        2. Copied from ``` https://github.com/ToyotaResearchInstitute/velodyne_simulator ```
* interactive_map_tester:
    1. Contains the nodes for saving the map and interactive map
    2. Contains the nodes for loading/displaying the map & interactive map
* pointcloud2_filter:
    1. Creates a particle filter that filters the rays that touch the ground
    2. Developed using : ```https://github.com/ros-perception/perception_pcl/issues/323 ```
    
    [TODO] i.  Contains a demo using the interactive map
    
    [TODO] ii. Add map save/load path as input


## Launch the simulation

### Autonomous exploration mode

* Terminal 1:
    ```
    source /opt/ros/foxy/setup.bash
    cd ~/colcon_ws/
    [if needed] source /opt/ros/foxy/setup.bash
    [optionally] colcon build --symlink-install --packages-select <package-name>
    source install/setup.bash
    ros2 launch autonomous_exploration launch_turtlebot3_simulation.launch.py [optionally gui:=false to hide gazebo]
    ```
* Terminal 2:
    ```
    cd ~/colcon_ws/
    source install/setup.bash
    ros2 launch autonomous_exploration simpleRobot
    ```
* Publish goal to the navigation controller
    ```
    ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
* Start the connection with Unity
    * Terminal 1:
    ```
    cd ~/catkin_ws
    [if needed] source /opt/ros/noetic/setup.bash
    [optionally] catkin build
    source devel/setup.bash
    roslaunch ros_unity launch_ros_unity_connection.launch
    ```
    * Terminal 2:
    ```
    cd ~/bridge_ws
    [if needed] source /opt/ros/foxy/setup.bash
    [optionally] colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
    source devel/setup.bash
    ros2 run ros1_bridge parameter_bridge
    ```
    * Start the unity app

* Save the maps :
    ```
        ros2 run interactive_map_tester saveInteactiveMap --ros-args -p maps_folder:=/home/antony/colcon_ws/src/thesis_ros2/maps/new_house 
    ```

### Use the interactive map to move around

* Terminal 1 - Launch the simulation:
```
    cd ~/colcon_ws/
    source install/setup.bash
    ros2 launch interactive_map_tester launchInteractiveMapTester.launch.py maps_folder:=/home/antony/colcon_ws/src/thesis_ros2/maps
```

## Transport additional messages between ROS2 and ROS

* Add the new message file on the package *autonomous_exploration_msgs*
* Add the new message file on the package *ros_unity_msgs* on the catkin_ws
* Add the new message names and pkgs on the file *autonomous_exploration_msgs/params/mapping_rules.yaml*, on the colcon_ws
* Build the autonomous_exploration_msgs package ```colcon build --symlink-install --packages-select autonomous_exploration_msgs```
* Build the ros1_bridge following the instructions above.
* Add a new entry on the file *ros_unity/config/rosbridge_params.yaml*, on the catkin_ws, containing the name of the topic, message type and queue
instructed the rosbridge to transport it from ROS2 to ROS1 and vice-versa
* Compile the bridge using ```colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure```
* If an error occurs try again ``` colcon build --symlink-install --packages-select ros1_bridge ```
* To send the new message to unity update the file *ros_unity/src/server_endpoint.py*, on the catkin_ws, by adding a new entry on the *tcp_server.start*
 part of the code


## Configure Unity

 * Detailed instructions can be found at : https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration
 1. Launch the unity application
 2. Add the robotics package on Unity ```https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md```

## Save/Load the map

* Run the map saver server
    ``` ros2 launch nav2_map_server map_saver_server.launch.py ```
* Save the map :
    ``` ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args --remap map:=/map ```
* Save the interactive map
    ``` ros2 run nav2_map_server map_saver_cli -f ~/interactive_map --ros-args --remap map:=/interactive_map/map ```
    
* Load the maps
    ``` ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}" ```

* Start the lifecycle node
    ```ros2 run nav2_util lifecycle_bringup map_server ```

### Used commands
* Saved the map
    ```ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args --remap map:=/map ```
* Saved interactive map
    ``` ros2 run interactive_map_tester saveInteactiveMap ```
* Create a rosbag with the map and robot_pos messages
    ``` ros2 bag record /rosbridge_msgs_publisher/map /rosbridge_msgs_publisher/robot_pos ```
* Start the pointcloud2 filter node
    ``` ros2 run pointcloud2_filter pcl_filter --ros-args -p hBeams:=1875 -p vBeams:=16 ```
* Use the ros1_bridge for all the topics
    ``` ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics ```

## Troubleshooting - Possible issues

* *AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT' This might be a ROS 1 message type but it should be a ROS 2 message type. Make sure to source your ROS 2 workspace after your ROS 1 workspace.* 
    Solution : https://github.com/ros2/ros2/issues/451 
* Change file permissions 
```sudo chmod -R o+rw docker-workspace/```
* Errors while compiling the ros1_bridge : ``` https://github.com/ros2/rosbag2_bag_v2/issues/32 ```

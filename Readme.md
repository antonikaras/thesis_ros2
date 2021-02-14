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
* thesis_gazebo:
    1. Contains the worlds and models used in the simulation
    2. Gazebo environments from ```https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps.git``` were used

## Launch the simulation

* Terminal 1:
    ```
    source /opt/ros/foxy/setup.bash
    cd ~/colcon_ws/
    [if needed] source /opt/ros/foxy/setup.bash
    [optionally] colcon build --symlink-install --packages-select <package-name>
    src2
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

## Transport additional messages between ROS2 and ROS

* Add the new message file on the package *autonomous_exploration_msgs*
* Add the new message file on the package *ros_unity_msgs* on the catkin_ws
* Add the new message names and pkgs on the file *autonomous_exploration_msgs/params/mapping_rules.yaml*, on the colcon_ws
* Build the autonomous_exploration_msgs package ```colcon build --symlink-install --packages-select autonomous_exploration_msgs```
* Build the ros1_bridge following the instructions above.
* Add a new entry on the file *ros_unity/config/rosbridge_params.yaml*, on the catkin_ws, containing the name of the topic, message type and queue
instructed the rosbridge to transport it from ROS2 to ROS1 and vice-versa
* To send the new message to unity update the file *ros_unity/src/server_endpoint.py*, on the catkin_ws, by adding a new entry on the *tcp_server.start*
 part of the code

# ROS2 turtlebot3 autonomous navigation

## Project packages

*   autonomous_exploration:
    1. Contains the autonomous exploration action server
    2. Handles the autonomous exploration using the new exploration targets detected by the specified frontier detection method
        i. Currently only vision based frontier detection is implemented
    3. Contains the simple robot api which is used to ease communication with the robot
* autonomous_exploration_msgs:
    1. Contains the message file definitions used by the frontier detection method
    2. Contains the action file definitions used by the autonomous exploration action 
* frontier_detectionvision:
    1. Contains the vision based frontier detection algorithm
* thesis_gazebo:
    1. Contains the worlds and models used in the simulation
    2. Gazebo environments from ```https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps.git``` were used

## Launch the simulation

* Terminal 1:
    ```
    source /opt/ros/foxy/setup.bash
    cd ~/colcon_ws/
    colcon build --symlink-install --packages-select <package-name>
    src2
    ros2 launch autonomous_exploration launch_turtlebot3_simulation.launch.py
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
    ```
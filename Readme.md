# ROS2 turtlebot3 autonomous navigation

## Gazebo environments

* Gazebo environments from ```https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps.git``` were used

## Launch the simulation

* Terminal 1:
    ```
    source /opt/ros/foxy/setup.bash
    cd ~/colcon_ws/
    colcon build --symlink-install --packages-select <package-name>
    src2
    ros2 launch vision_based_exploration launch_turtlebot3_simulation.launch.py
    ```
* Publish goal to the navigation controller
    ```
    ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
    ```
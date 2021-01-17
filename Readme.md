# ROS2 turtlebot3 autonomous navigation

## Launch the simulation

* Terminal 1:
    ```
    cd ~/colcon_ws/
    colcon build --symlink-install
    src2
    ros2 launch thesis_ros2 launch_turtlebot3_simulation.launch.py
    ```
* Publish goal to the navigation controller
    ```
    ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
    ```
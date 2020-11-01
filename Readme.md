# ROS2 turtlebot3 autonomous navigation

## Install the turtlebot3 packages

```
cd docker-workspace
mkdir -p colcon_ws/src
cd colcon_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
```
* Open the turtlebot3.repos file and change Git from ros2 to foxy-devel 
```
vcs import src < turtlebot3.repos
colcon build --symlink-install
```

## Build the docker image

```
docker build --tag thesis_ros2 . --rm
```

## Clean the <none> docker images
```
docker system prune
```

## Run the docker image
* Terminal 1
```
./run-docker-image-gpu.sh
bt-ws
```
* Terminal 2
```
docker exec -it thesis_ros2-tester bash
bt-ws
```
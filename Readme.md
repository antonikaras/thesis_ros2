# ROS2 turtlebot3 autonomous navigation

## Install the turtlebot3 packages

```
git submodule update --init --recursive
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
bws
```
* Terminal 2
```
docker exec -it thesis_ros2-tester bash
cws
src2
```
* Change the permissions on the colcon_ws
```
sudo chown -R $USER docker-workspace
```
# ROS2 package for keyestudio mini-tank

# Run the docker container:
```bash
docker run -it --net=host --privileged ros_humble
```

# Open another terminal window in the same docker container
## first get the container name
```bash
docker ps
```
## this will show something like "elegant_fox" or something like that
### now run:
```bash
docker exec -it elegant_fox /bin/bash
```

# save the docker container 
```bash
docke commit elegant_fox
```

# Run a ros2 launch file
```bash
ros2 launch mini_tank_ros2 your_launch_file_name
```

# Run a ros2 node
```bash
ros2 run mini_tank_ros2 your_node_name
```

# Update your package after you've made changes to a node
```bash
cd src/mini_tank_ros2
git init
git pull origin main
cd ../..
colcon build --packages-select mini_tank_ros2
source install/setup.bash
```

# See what topics are being published
```bash 
ros2 topic list
```

# Show the messages the topics are publishing
```bash
ros2 topic echo /your_topic_name
```

# Show the nodes that are running
```bash
ros2 node list
```


# SimBot Explorer – ROS Noetic + Gazebo Learning Project

Fundamental implementation of ROS using a simulated TurtleBot3 robot in Gazebo. The project guides you through creating your own ROS nodes, publishers, subscribers, and more while observing the robot's behavior in a virtual environment.

---

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- TurtleBot3 packages

Install TurtleBot3 and Gazebo packages:

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3-gazebo
```

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
### Workspace setup
create a new catkin workspace and package

```bash
mkdir -p ~/ros_sim_ws/src
cd ~/ros_sim_ws/src
catkin_create_pkg simbot_explorer rospy std_msgs geometry_msgs sensor_msgs
cd ~/ros_sim_ws
catkin_make
source devel/setup.bash
```
### Launch Simulation

Create a launch directory inside your package:
```bash
mkdir -p ~/ros_sim_ws/src/simbot_explorer/launch
```

Create the laucnh file:
```xml
<launch>
    <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_house.launch" />
</launch>
```

Run the simulation:
```bash
roslaunch simbot_explorer simulation.launch
```

### ROS Node 1: teleop_node.py (Publisher)
Publishes velocity commands to move the robot forward while turning.

`File: scripts/teleop_node.py`

### ROS Node 2: laser_listener.py (Subscriber)
Subscribes to /scan topic and prints the distance to obstacles in front.

`File: scripts/laser_listener.py`

make sure it is execuatable using `chmod  +x scripts/teleop_node.py` and `chmod +x scripts/laser_listner.py`

Then run both the scripts using `rosrun`.



## ToDO

- Add stop_service node to halt robot via ROS service
- Add explorer.launch to start simulation and all nodes with parameters
- Add obstacle_avoidance node using reactive LIDAR-based steering
- Add RViz config to visualize LIDAR and robot pose
- Add waypoint navigation logic with pose tracking via /odom

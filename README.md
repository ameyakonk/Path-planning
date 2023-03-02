# Path-planning

## Overview
The repository contains the following contents.

1. ROS Packages and source code to run the BFS, Dijkstra, A* and RRT* path planner and goal assignment solvers.
2. Video files suggesting node exploration and finding optimal path to goal in Gazebo and Python

## Personnel
### Ameya Konkar 

UID:118191058

Master's Student at University of Maryland, College Park

## Dependencies 

1. Python 3
2. Numpy
3. Matplotlib
4. ROS
5. Gazebo
6. Scipy

### Building the Program and Tests

```
sudo apt-get install git
git clone --recursive https://github.com/ameyakonk/Path-planning.git
```
To Run the code:
```
cd catkin workspace/ 
catkin build
roslaunch <specific folder> turtlebot3_autorace.launch

```

# Tu Many Robots Project for Project@TUM
## Project introduction
With this project we build a multi agent robotics system which allows multiple robots to find a blind robot (i.e a robot with disabled or no sensors) and a goal in an unknown environment. By working together the robots can find themselves in their surroundings and share knowledge gained about their surroundings. After locating the goal, the blind robot and being able to compute a path, the robots with sensors will guide and lead the blind robot to the goal.


# How to run it


## System Prerequisites
This project was built with ROS Noetic on Ubuntu 20.04.

This project works with two computer setups. Either you are able to run our expensive Computer Vision pipeline which allows us to locate the blind robot and goal with a computer vision pipeline. For this you need to have a graphics card. Testing was done with an NVIDIA 2060 Super with a AMD Ryzen 7 3700X. Because the simulation runs 3 robots and 2 visual pipelines the visualization of results is computationally expensive and affects performance while running the complete system.

If your setup does not have these capabilites we have an option to forgoe this computational complexity by using a mock version of our visual pipeline which does not need a GPU to work. Since the data gets projected from simulation the results are better than using the real system but more on that will be explained in the relevant section later on.

@TODO CV Stuff


## Installation Instructions
The installation for this project is quite straightforward for the user. Since we are using robotino robots and were provided with a functioning installation, our implementation extends this installation with all the needed packages to run our software.  
Copy the install_script.bash file onto your system and run it, this completely installs all the needed packages for our software to work on your system, including this github project.


## Build Instructions
The OpenCV module can be hard to get running, see the readme in the related module.
The topics from this module can be mocked, however, and to ensure the package builds please use

- catkin config --blacklist darknet_ros
- catkin build


## Starting the simulation

Note! To setup the robot positions,

go to "catkin_workspace"/src/TU_Many_Bots
```
source config_script.bash
```
This will give a printout of the configured environment.

The map can be changed by
export tmb_ROBOT_ENV=[simple_corridor, maze, maze_simple, maze_simple_2, maze_clutter]
then running the config script again. The main maps are simple_corridor and maze_clutter.

The most important commands to get the simulation to start:
```
roslaunch tmb_startup complete_launch.launch
```

In addition, the mock openCV module is run with:
```
rosrun tmb_perception target_distance_detector
```
while the real module is run with
```
roslaunch darknet_ros darknet_ros.launch
```

Topics:

Node: target_distance_detector
Publishes:
* '/tmb_perception/object_sighted', of custom type: Object_Sighted


# How it works
@@ Picture of the nodes 

We use a multitude of prebuilt and custom nodes and packages to accomplish our goal specified in the introduction. 

@@ Visualization of packages

## Overall explanation

## Visual Pipeline Real

## Visual Pipeline Mock

## SLAM

## map_merge

## exploration

## move_base

## position listener

## following routine

## robot_state_publisher

## Possible further development
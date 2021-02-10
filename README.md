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
### Visual pipeline
To perceive the environment we use RGB-D cameras and LiDar sensors. The cameras allow us to classify and position the blind robot and the goal, which we can use to publish the positions on ros topics to use them later on. These topics will be used further on to start the guidance procedure of the robot to guide the blind robot to the goal.

@@ yolo ros stuff

Since we do not do any preprocessing to laser data the data gets processed in a raw form by the chosen SLAM algorithm. 

### SLAM
For Simultaneous Localization and Mapping (SLAM) we use the standard gmapping package. Configuration files are in the config folder in the @@ package. Gmapping was chosen over the slam-toolbox online async algorithm after evaluating both algorithms in testing. Even after including more cluttering to the maps, the toolbox still had trouble providing twist free maps and it was also getting lost during loop closures. Gmapping has the distinct advantage that our map merge algorithm works best with maps with fixed sizes which are provided by gmapping. With map merging we are able to compute an overall map which gets explored by both robots where we can locate the blind robot in.

### Path Planning
Path planning works in two different phases. First we explore the environment by finding unknown space and creating frontiers, this is done by the explore-lite package. We then publish a goal while trying to explore the biggest frontiers. The path is computed by the move_base package by computing a global costmap on the robot maps and then utilizing the laser sensors to perceive the immediate environment and adjust to dynamic obstacles we use the local costmap. 

After perceiving the goal, the blind robot and being able to compute a path. We switch to the guiding routine which disables the exploration and allows the robots to move to the blind robot and guide it to the goal. The path planning works similarly like before but the goal publishing nodes change.

### Guiding routine
@@

## Visual Pipeline Real

## Visual Pipeline Mock

## SLAM
We use gmapping with a largely base setup. We changed the parameters so the map gets updated at a rate of 1Hz. Space over 5 meters away gets classified as unknown space which allows to compute frontiers in exploration. 

## map_merge
We use the multirobot_map_merge package provided by @@. This allows us to merge maps where the robot start positions are known. In theory the algorithm is also able to compute maps without knowing the start positions of the robot. This did not work in practice but we could overlay with known starting positions anyway. For known start positions the maps get overlayed. This means that deviations in SLAM lead to large deviations in the computed merged map. So a good SLAM is crucial for this to work properly.

## exploration
The only adjustments to the algorithm is a change in topics and increasing the timeout period so frontiers get only classified as unreachable after a longer peroid of time. The algorithm tracks unknown space in the provided map to compute frontiers. Then by computing the biggest frontier, a goal is published on the specified topic and then we use move_base to explore the environment. Map updates lead to new frontiers, which will then impact the computed goal so the biggest frontiers get explored first in a greedy approach. 

## move_base

## position listener

## following routine

## robot_state_publisher

# API 

## Possible further development
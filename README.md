# Tu Many Robots Project for Project@TUM
With this project we build a multi agent robotics system which allows multiple robots to find an object, a blind robot (i.e a robot with disabled or no sensors) and a goal in an unknown environment. The blind robot is capable of transporting the object while the other robots are only fitted with sensors to map their surroundings and therefore lack this ability. By working together the robots can find themselves in their surroundings, share knowledge gained about their surroundings and communicate to accomplish their goal.

## Installation Instructions
The installation for this project is quite straightforward for the user. Since we are using robotion robots and were provided with a functioning installation, our implementation extends this installation with all the needed packages to run our software.  
Copy the install_script.bash file onto your system and run it, this completely installs all the needed packages for our software to work on your system.


## Build Instructions
The OpenCV module can be hard to get running, see the readme in the related module.
The topics from this module can be mocked, however, and to ensure the packaged builds please

- catkin config --blacklist darknet_ros
- catkin build




## Starting the simulation

Note! To setup the robot positions,

go to src/TU_Many_Bots
```
source config_script.bash
```
This will give a printout of the configured environment.
The map can be changed by
export robot_env=[simple_corridor, maze, maze_simple, maze_simple_2]
then running the config script again.
It's possible that the maze_simple_2 has invalid starting positions, just change them on the config script and add pr thanks!

The most important commands to get the simulation to start:
```
roslaunch tmb_startup complete_launch.launch
```

In addition, the mock openCV module is run with:
```
rosrun tmb_perception target_distance_detector
```

Topics:

Node: target_distance_detector
Publishes:
* '/tmb_perception/object_sighted', of custom type: Object_Sighted

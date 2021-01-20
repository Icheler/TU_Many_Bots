# Tu Many Robots Project for Project@TUM
With this project we build a multi agent robotics system which allows multiple robots to find an object, a blind robot (i.e a robot with disabled or no sensors) and a goal in an unknown environment. The blind robot is capable of transporting the object while the other robots are only fitted with sensors to map their surroundings and therefore lack this ability. By working together the robots can find themselves in their surroundings, share knowledge gained about their surroundings and communicate to accomplish their goal. 

## Installation Instructions
The installation for this project is quite straightforward for the user. Since we are using robotion robots and were provided with a functioning installation, our implementation extends this installation with all the needed packages to run our software.  
Copy the install_script.bash file onto your system and run it, this completely installs all the needed packages for our software to work on your system.

## Starting the simulation

The most important commands to get the simulation to start:
```
roslaunch tmb_communication complete_launch.launch
```

## Bugs or not wanted behavior:
* Robots dont follow computed paths
* Robots run into perceived obstacles
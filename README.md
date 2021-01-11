# Tu Many Robots Project for Project@TUM

The most important commands to get the simulation to start:
```
roslaunch tmb_communication robots_testing.launch
roslaunch tmb_communication multi_map_merge.launch
roslaunch tmb_communication move_base.launch
roslaunch tmb_communication explore_robots.launch
```
For a better overview of what the robots have perceived and are planning to do look at:
```
rosrun rviz rviz -d `rospack find tmb_communication`/config/rviz_basic_setup.rviz
```

Bugs or not wanted behavior:
* Robots dont follow computed paths
* Robots get lost easily and start recovery behavior
* Robots run into perceived obstacles